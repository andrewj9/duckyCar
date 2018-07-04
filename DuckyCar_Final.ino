
#include "Gyro.h"
#include "Timer.h"
#include "Object.h"
#include <Sonar.h>
#include <PWM_LIGHTS.h>
#include <MOTOR_CONTROL.h>
#include <PID.h>
#include <MemoryDelegate.hpp>
#include <SPI.h>
#include <Pixy.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

//Pin Definitions
#define PIN_A1 7	//H-Bridge pin A1
#define PIN_A2 8	//H-Bridge pin A2
#define PIN_B1 9	//H-Bridge pin B1
#define PIN_B2 A3	//H-Bridge pin B2
#define PIN_ENA 5	//H-Bridge enable A pin
#define PIN_ENB 6	//H-Bridge enable B pin
#define PIN_OE 3	//Output Enable for 16-channel PWM controller
#define TRIGGER_PIN A0
#define ECHO_PIN A1
#define LINE_SENSOR 2

//Servo Constant Definitions
#define SERVO_PIN 7	//Attached to 16-Channel PWM driver
#define speedOfSound 0.0344 // cm per uS
#define correctionFactor 0.00  // added to speed of sound to correct for sensor variance (found through experimentation)
#define OBSTACLE_THRESHOLD 20	// Distance from object in cm before obstacle detection is triggered
#define DEPTH_FIELD_SIZE 5 

//PWM Channel Definitions
#define LED_LFI 13		//Left Front Inside
#define LED_LFO 12		//Left Front Outside
#define LED_LRO 2		//Left Rear Outside
#define LED_LRI 3		//Left Rear Inside
#define LED_RFO 15		//Right Front Outside
#define LED_RFI 14		//Right Front Inside
#define LED_RRI 0		//Right Rear Inside
#define LED_RRO 1		//Right Rear Outside

//PWM Duty Cycle Definitions
#define PWM_ON 0	//Time each cycle to set pulse width high (
#define PWM_OFF 2048	//Time each cycle to set pulse width low
#define PWM_FREQ 50		//Frequency of PWM (Hz)

#define TARGET 80 //Width returned from camera
#define MOTOR_TIMEOUT 150 //Time after last reset until motors are stopped (milliseconds)
#define MOVETIME 1000	//Time to move vehicle
#define TURNTIME 650	//Time to turn vehicle
#define MINSPEED 115	//Minimum speed that will move the vehicle
#define MAXSPEED 185	//Max speed to drive the motors for forward or backward movement
#define CRUISESPEED 125	//Normal cruising speed for PID control
#define TURNSPEED 225	//Max speed to drive the motors for turning
#define BRAKESPEED 75	//For applying resistance to a motor to keep those wheels from rotating while other wheels turn

//PID Controller Definitions
#define KP 5  // Proportional Gain
#define KI 0  // Integral Gain
#define KD 0  // Derivative Gain

//PixyCam Signature Definitions
#define SKIPLINE 1
#define GREENLIGHT 2	//Subject to change
#define STOP 6	//Subject to change
#define RAMP 19 //Subject to change
#define GATE 28	//Subject to change
#define DUCKS 5	//Subject to change
#define TUNNEL 645	//Subject to change
#define GARAGE 39	//Subject to change

//PixyCam Parameter Definitions
#define CENTERX 160
#define CENTERY 120
#define X_OFFSET -75
#define Y_OFFSET 100
#define MAX_BLOCKS 20

#define TWEAKFACTOR	6	//adjustment for PID control for turning (Range: 0 - 11)

//Engage the prime directive
//bool engage = true;
//bool stopping = false;
//bool stopBar = false;
int urgent = 0;
//bool ignoreStopBar = true;
//bool ignoreSkipLines = false;
//bool ducks = false;

//Bit flags for determining the state of the environment
//Least to most significant bit
//B0 = engage
#define engage 0
//B1 = stopping
#define stopping 1
//B2 = stopBar
#define stopBar 2
//B3 = ignoreStopBar
#define ignoreStopBar 3
//B4 = ignoreSkipLines
#define ignoreSkipLines 4
//B5 = ducks
#define ducks 5
//B6 = turning
#define turning 6
//B7 = british
#define british 7
byte flags = B00001000;

int turnCounter = 2;
int goneBritish = 0;
//Gyro
Gyro gyro = Gyro();
int turnAMT = 0;

//Last detected skipline position
int lastXPos = CENTERX;
int lastYPos = CENTERY;

//Create a PixyCam object
Pixy pixy;

//Create PID controllers for left and right motor controllers
PID lPID = PID(KP, KI, KD);
PID rPID = PID(KP, KI, KD);

//Create a motor controller for drive functionality
MOTOR_CONTROL motors = MOTOR_CONTROL(PIN_A1, PIN_A2, PIN_B1, PIN_B2, PIN_ENA, PIN_ENB);

//Create a PWM controller for effects lighting
PWM_LIGHTS lights = PWM_LIGHTS(PIN_OE, LED_LFI, LED_LFO, LED_LRO, LED_LRI, LED_RFO, LED_RFI, LED_RRI, LED_RRO);

//Create an ultasonic range sensor/servo object
Sonar sonar = Sonar(SERVO_PIN, TRIGGER_PIN, ECHO_PIN, speedOfSound, correctionFactor);

//Create a null object (Ignored by object recognition functions)
Object nullObject = Object(-1);

//Initialize detected object arrays with null objects
Object detectedBlocks[MAX_BLOCKS] = {nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject, nullObject };

//Create timeout timers for various purposes
Timer motorTimer = Timer(MOTOR_TIMEOUT);
Timer loopTimer = Timer(100);
Timer pixyTimer = Timer(20);
Timer turnTimer = Timer(100);
Timer interruptTimer = Timer(10);
Timer printTimer = Timer(1000);
Timer rampTimer = Timer(3000);
Timer rampCleared = Timer(4000);
Timer gateTimer = Timer(1000);
Timer lineTimer = Timer(1000);

void setup() {

	//Start the serial port
	Serial.begin(256000);
	Serial.println("Serial Enabled");

	//16-Channel PWM controller setup
	lights.init();
	//Serial.println("PWM Active");
	lights.setParams(PWM_ON, PWM_OFF, PWM_FREQ);
	//Serial.println("PWM Frequency Set");
	lights.enable(true);
	//Serial.println("PWM Output Disabled");

	//PWM Controller Power-up
	pwmToggle();
	//Serial.println("PWM Active");

	//Init Sonar
	sonar.init();
	//Serial.println("Sonar Active");

	//Init Camera
	pixy.init();
	Serial.println("Camera Active");

	//PID Limit Definition
	lPID.setMaxOutput(MAXSPEED);
	rPID.setMaxOutput(MAXSPEED);

	//Fill null object with "null" values (-1)
	nullObject.createNull();

	//Initialize the gyroscope object
	gyro.init();
	Serial.println("Gyroscope Active");

	//Initialize the line sensor
	attachInterrupt(digitalPinToInterrupt(LINE_SENSOR), lineSensed, RISING);
	bitSet(flags, engage);
}

//This is where the magic happens
void loop() {
	switch (bitRead(flags, engage)) {
		case 0:	
			if (bitRead(flags, turning)) turn(turnAMT);
			else motorStop(); break;
		case 1:	proceed(); break;
		default: break;
	}
	breakfast();
}

void lineSensed() {
	if (!interruptTimer.timeout())return;
	//if (bitRead(flags, ignoreStopBar)) return;
	interruptTimer.reset();
	bitSet(flags, stopBar);
}

//Set all motor PWM width to zero
void motorStop() {
	if (!motorTimer.timeout()) return;
	else {
		eStop();
	}
}

//Immediate Stop
void eStop() {
	motors.stop();
	lights.allOff();
}

//Drive forward based on max speed setting
void forward(int speed) {
	switch (speed) {
	case 1:
		motors.drive(-MINSPEED, -MINSPEED);
		break;
	case 2:
		motors.drive((-MINSPEED - MAXSPEED)*0.5, (-MINSPEED - MAXSPEED)*0.5);
		break;

	case 3:
		motors.drive(-MAXSPEED, -MAXSPEED);
		break;
	default:
		break;
	}
	lights.drive(-MAXSPEED, -MAXSPEED);
}

//Drive backward based on max speed setting
void back() {
	motors.drive(MAXSPEED, MAXSPEED);
	lights.drive(MAXSPEED, MAXSPEED);
}

//Turn left based on max turn speed setting
void left() {
	turn(90);
	//motors.drive(TURNSPEED, -TURNSPEED);
	lights.drive(TURNSPEED, -TURNSPEED);
}

//Turn right based on max turn speed setting
void right() {
	turn(-90);
	//motors.drive(-TURNSPEED, TURNSPEED);
	lights.drive(-TURNSPEED, TURNSPEED);
}

//Slow down the left wheels, but not full reverse for a more gradual left turn
void leftBrake() {
	motors.drive(BRAKESPEED, -TURNSPEED);
	lights.drive(BRAKESPEED, -TURNSPEED);
}

//Slow down the right wheels, but not full reverse for a more gradual right turn
void rightBrake() {
	motors.drive(-TURNSPEED, BRAKESPEED);
	lights.drive(-TURNSPEED, BRAKESPEED);
}

//Serial Interpreter (to replace getCommand() eventually)
void breakfast() {
	char getstr = Serial.read();
	switch (getstr) {
	case 'f':	motorTimer.reset(); forward(2);	 break;
	case 'b':	motorTimer.reset(); back();	break;
	case 'l':	turnAMT = 10; left();	break;
	case 'r':	turnAMT = -10; right();	break;
	case 's':	stopSign();   break;
	case 'g':	stateChange(); break;
	case 'i':	pwmToggle(); break;
	case 'w':
		updateBlocks();
		Serial.println(getObject(GATE).xPos());
		break;
	case 'n':
		updateBlocks();
		Serial.println(getBlocks(SKIPLINE));
		break;
	case 'd':	Serial.println(sonar.getDistance());	break;
	case 'c':	sonar.checkServo();	break;
	case 'o':
		updateBlocks();
		Serial.println(getObject(GATE).width());
		break;
	default:  break;
	}
}

//Engage or disengage the navigation sequence
void stateChange() {
	if (bitRead(flags, engage)) bitClear(flags, engage);
	else bitSet(flags, engage);
	bitClear(flags, stopping);
	if (bitRead(flags, engage)) Serial.println("Engaged");
	else Serial.println("Disengaged");
}

//Toggle the output enable pin of the 16-channel PWM controller
void pwmToggle() {
	bool state = lights.isEnabled();
	Serial.print("PWM Output: ");
	Serial.println(state);
	//PWM Off Indicator
	if (!state) {
		lights.offSequence();
	}
	lights.enable(!state);
	state = lights.isEnabled();
	//PWM On Indicator
	if (!state) {
		lights.onSequence();
	}
}

//Poll the camera for detected blocks and populate an array with objects
int updateBlocks() {
	if (pixyTimer.timeout()) {
		uint16_t blocks = 0;
		blocks = pixy.getBlocks();
		if (blocks <= 2) blocks = pixy.getBlocks();
		//Serial.println(blocks);
		pixyTimer.reset();
		if (blocks) {
			if (blocks > MAX_BLOCKS) blocks = MAX_BLOCKS;
			for (int i = 0; i < blocks; i++) {
				detectedBlocks[i] = Object(pixy.blocks[i].signature);
				detectedBlocks[i].update(pixy.blocks[i].x, pixy.blocks[i].y, pixy.blocks[i].angle, pixy.blocks[i].height, pixy.blocks[i].width);
				//Serial.print(detectedBlocks[i].sig());
				//Serial.print(':');
			}
			//Serial.println();
			if (blocks < MAX_BLOCKS) {
				for (int i = blocks; i < MAX_BLOCKS; i++) {
					detectedBlocks[i] = nullObject;
				}
			}
			return 1;
		}
	}
	return 0;
}

//Return the first index of the block with the specified signature
int getBlocks(int sig) {
	for (int i = 0; i < MAX_BLOCKS; i++) {
		if (detectedBlocks[i].sig() == sig) {
			return i;
		}
	}
	return -1;
}

//Return the first detected object of the specified signature
Object getObject(int sig) {
	for (int i = 0; i < MAX_BLOCKS; i++) {
		if (detectedBlocks[i].sig() == sig) {
			return detectedBlocks[i];
		}
	}
	return nullObject;
}

//Drive function for PID control
void goToTarget(int lGain, int rGain) {
	motors.drive(lGain, rGain); //Send the drive signal
	lights.drive(lGain, rGain);	//Make the lights flash
}

//PID-controlled drive method
void followTheYellowBrickRoad() {
	int tweak = 0;
		int xAve = 0;
		int yAve = 0;
		int numAve = 0;
		for (int i = 0; i < MAX_BLOCKS; i++) {
			if (detectedBlocks[i].sig() == SKIPLINE && detectedBlocks[i].yPos() > CENTERY) {
				xAve += detectedBlocks[i].xPos();
				yAve += detectedBlocks[i].yPos();
				numAve++;
				if (detectedBlocks[i].yPos() > lastYPos) {
					lastXPos = detectedBlocks[i].xPos();
					lastYPos = detectedBlocks[i].yPos();
				}
			}
		}
		int setPos = 0;
		if (bitRead(flags, british) == 1) setPos = CENTERX - X_OFFSET;
		else setPos = CENTERX + X_OFFSET;
		if (numAve == 0) numAve = 1;
		xAve /= numAve;
		int lBoost = 0;
		//if (xAve < setPos) lBoost = 50;
		//if (abs(xAve - CENTERX) < 20) xAve += 50;
		tweak = (setPos - xAve)*TWEAKFACTOR / 11;
		//if (yAve < CENTERX + Y_OFFSET) tweak /= 2;
		int error = 50;
		if (bitRead(flags, engage)) {
			int lSig = -error + tweak;
			int rSig = -error - tweak;
			goToTarget(lPID.getControl(lSig + lBoost), rPID.getControl(rSig));	//Send the error and tweak values to the PID controllers and drive to the target
			motorTimer.reset();
		}
		//else motorStop();
}	

void proceed() {
	if (bitRead(flags, turning)) turn(turnAMT);
	else if (updateBlocks()) {
		if (bitRead(flags, ducks)) {
			int d = 0;
			for (int i = 0; i < MAX_BLOCKS; i++) {
				if (detectedBlocks[i].sig() == DUCKS) {
					if (detectedBlocks[i].yPos() > CENTERY) {
						d++;
						interpretSignature(detectedBlocks[i]);
					}
				}
				if (d == 0) bitClear(flags, ducks);
			}
		}
		else if (bitRead(flags, stopping) == 0) {
			urgent = 0;
			bitClear(flags, ignoreSkipLines);
			for (int i = 0; i < MAX_BLOCKS; i++) {
				if (detectedBlocks[i].sig() == DUCKS) {
					if (detectedBlocks[i].yPos() > CENTERY - 50) {
						bitSet(flags, ducks);
						interpretSignature(detectedBlocks[i]);
					}
					break;
				}
			}
				for (int i = 0; i < MAX_BLOCKS; i++) {
					if (bitRead(flags, DUCKS) == 1) return;
					if (detectedBlocks[i].sig() == STOP) {
						if (abs(detectedBlocks[i].xPos() - CENTERX) < 50 && CENTERY - detectedBlocks[i].yPos() < 60) {
							bitSet(flags, ignoreSkipLines);
							bitClear(flags, ignoreStopBar);
							forward(1);
						}
						if (bitRead(flags, stopBar)) {
							bitSet(flags, stopping);
							eStop();
							break;
						}
					}
				}

				if (gateTimer.timeout()) {
					bitClear(flags, british);
					goneBritish = 0;
				}
				for (int i = 0; i < MAX_BLOCKS; i++) {
					if (detectedBlocks[i].sig() == GATE) {
						interpretSignature(detectedBlocks[i]);
						break;
					}
				}
				for (int i = 0; i < MAX_BLOCKS; i++) {
					if (detectedBlocks[i].sig() == GARAGE) {
						interpretSignature(detectedBlocks[i]);
						break;
					}
				}
				int skipseen = 0;
				for (int i = 0; i < MAX_BLOCKS; i++) {
					if (bitRead(flags, ignoreSkipLines)) break;
					if (detectedBlocks[i].sig() == SKIPLINE) {
						skipseen++;
						interpretSignature(detectedBlocks[i]);
						break;
					}
					
				}
				if (lineTimer.timeout()) bitClear(flags, ignoreSkipLines);
				if (bitRead(flags, ignoreSkipLines)) return;
				if (rampTimer.timeout() && skipseen == 0) turnSequence();
				for (int i = 0; i < MAX_BLOCKS; i++) {
					if (detectedBlocks[i].sig() == RAMP) {
						interpretSignature(detectedBlocks[i]);
						break;
					}
				}
		}
		else {
			for (int i = 0; i < MAX_BLOCKS; i++) {
				if (detectedBlocks[i].sig() == STOP) {
					break;
				}
				else if (detectedBlocks[i].sig() == GREENLIGHT) {
					if (abs(detectedBlocks[i].xPos() - CENTERX) < 50 && CENTERY - detectedBlocks[i].yPos() < 50) {
						bitClear(flags, ignoreSkipLines);
						bitSet(flags, ignoreStopBar);
						bitClear(flags, stopping);
						forward(2);
						delay(1000);
						break;
					}
				}
			}
		}
	}
}


//Turn the amount of the specified angle using the gyroscope
bool turn(int angle) {
	Serial.println("Turning");
	if (bitRead(flags, turning) == 0) {
		bitSet(flags, turning);
		gyro.resetTurned();
		gyro.setTimeZero();
		turnAMT = angle;
	}
	float deg = gyro.getTurned();
	if (abs(deg) < abs(turnAMT)) {
		if (angle > 0) {
			motors.drive(TURNSPEED, -TURNSPEED);
			lights.drive(TURNSPEED, -TURNSPEED);
		}
		else {
			motors.drive(-TURNSPEED, TURNSPEED);
			lights.drive(-TURNSPEED, TURNSPEED);
		}
		return true;
	}
	else {
		bitClear(flags, turning);
		turnAMT = 0;
		gyro.resetTurned();
		return false;
	}
}

//Use the distance sensor to check for obstacles for local autonomous navigation
void navigate() {
	bool obstacle = obstacleDetect();
	if (obstacle) {
		sonar.sweep(obstacle);
		int heading = sonar.lookSweep();
		int headingAngles[] = { -90, -45, 0, 45, 90 };
		while (turn(headingAngles[heading]));
	}
	else {
		proceed();
	}
}

bool obstacleDetect() {
	int dist = sonar.getDistance();
	if (dist < OBSTACLE_THRESHOLD) {
		eStop();
		return true;
	}
	else
	{
		return false;
	}
}

//Figure out what the signature of the detected block and make a decision based on that.
//Case numbers will be tweaked based on how the color codes are programmed into the pixy.
//The current numbering scheme is acting as placeholder
void interpretSignature(Object block) {
	switch (block.sig()) {
	case SKIPLINE:	//Skipline is seen
		if (printTimer.timeout()) {
			Serial.println("Skipline");
			printTimer.reset();
		}
		followTheYellowBrickRoad();
		break;
	case STOP:	//Stoplight/stopsign is seen
		if (printTimer.timeout()) {
			Serial.println("Stop");
			printTimer.reset();
		}
		//turnSequence();
		break;
	case DUCKS:	//Ducks are seen
		if (printTimer.timeout()) {
			Serial.println("Ducks");
			printTimer.reset();
		}
		if (abs(block.xPos() - CENTERX) < 140) bitSet(flags, ducks);
		else bitClear(flags, ducks);
		if (bitRead(flags, ducks)) {
			//back();
			//delay(20);
			eStop();
		}
		break;
	case RAMP: //The ramp is seen
		if (printTimer.timeout()) {
			Serial.println("Ramp");
			printTimer.reset();
		}
		//ramp();
		break;
	case GATE:	//The gate is seen
		if (printTimer.timeout()) {
			Serial.println("Gate");
			printTimer.reset();
		}
		gate();
		break;
	case GARAGE: //The garage is seen
		garage();
		break;
	default:
		break;
	}
}

//Action to stop at stop line
void stopSign() {
	//When stop sign is identified, pull forward to stop bar using line sensors
	//Pause for a moment
	//Proceed
	//Serial.println("Stop Sign");
	if (bitRead(flags, stopping)) bitClear(flags, ignoreStopBar);
	if (!bitRead(flags, stopBar)) {
		bitSet(flags, ignoreSkipLines);
		forward(1);
		return;
	}
	eStop();
	lights.allRed();
	bitSet(flags, ignoreStopBar);
	delay(1000);
	bitClear(flags, stopBar);
	lights.allOff();
	bitClear(flags, stopping);
	bitClear(flags, ignoreSkipLines);
	forward(2);
	delay(1000);
}

//Action to wait for stoplight
void stopLight() {
	//When stop light is identified, pull forward to stop bar using line sensors
	//Wait for "light" to turn green
	//Proceed
	//Serial.println("Stop Light");
	eStop();
	lights.allRed();
	bitSet(flags, ignoreStopBar);
}

//Action to drive over the ramp
void ramp() {
	//When ramp is identified, increase speed, drive directly at signature,
	//Start following the skip line again
	rampTimer.reset();
	setPosition(RAMP, CENTERX + 30);
	delay(1000);
}

//Action to drive thought the gate
void gate() {
	//Drive toward the signature
	Object target = Object(-1);
	bitSet(flags, ignoreSkipLines);
	lineTimer.reset();
	for (int i = 0; i < MAX_BLOCKS; i++) {
		//Assume that the open side of the gate will have a signature less than 0�
		if (detectedBlocks[i].sig() == GATE) {
			if (detectedBlocks[i].phi() < 0) {
				if (abs(CENTERX - detectedBlocks[i].xPos()) < 30) {
					if (detectedBlocks[i].width() > 18) {
						if (goneBritish == 0) {
							eStop();
							delay(200);
							while(turn(65));
							forward(2);
							delay(600);
							while(turn(-65));
							forward(2);
							delay(1000);
							goneBritish++;
						}
						bitSet(flags, british);
						gateTimer.reset();
					}
				}
			}
			else {
				if (abs(CENTERX - detectedBlocks[i].xPos()) < 30) {
					if (detectedBlocks[i].width() > 15) {
						forward(2);
						delay(1000);
						}
						gateTimer.reset();
					}
				}
			}
		if (detectedBlocks[i].sig() == SKIPLINE) bitClear(flags, ignoreSkipLines);
	}
}

//Action to drive through the tunnel
void tunnel() {
	//Engage sonar to navigate through the tunnel Maybe
	//Working on relocating sonar servo to the front of 
	//chassis hanging upside down so that the camera will 
	//have an unobstructed view from up high.  Testing
	//has not yet been conducted to determine if the sonar 
	//will be able to pivot.
}

//Action to parl in the garage
void garage() {
	//Use sonar to pull into garage and stop
	int target = 5;
	int distance = sonar.getDistance();
	int error = target - distance;
	if (abs(error) < 2) error = 0;
	int leftSignal = lPID.getControl(error);
	while (updateBlocks() > 0) {
		setPosition(GARAGE, CENTERX);
	}
	motors.drive(leftSignal, leftSignal);
	lights.drive(leftSignal, leftSignal);
}

//Adjust PID so that specified object is at the specified x-position
//This should work as the skipline following code did, but in a more generalized way
void setPosition(int sig, int xpos) {
	int xAve = 0;
	int numBlocks = 0;
	for (int i = 0; i < MAX_BLOCKS; i++) {
		if (detectedBlocks[i].sig() == sig) {
			xAve += detectedBlocks[i].xPos();
			numBlocks++;
		}
	}
	if (numBlocks > 0) xAve /= numBlocks;
	int tweak = (xpos - xAve)*TWEAKFACTOR / 11;
	int error = 10;
	if (bitRead(flags, engage)) {
		int lSig = -error + tweak;
		int rSig = -error - tweak;
		goToTarget(lPID.getControl(lSig), rPID.getControl(rSig));	//Send the error and tweak values to the PID controllers and drive to the target
		motorTimer.reset();
	}
}

void turnSequence() {
	switch (turnCounter) {
	case 0: {//Make a right turn to line up for the stoplight
		while (turn(-60));
		forward(2);
		//bitClear(flags, ignoreStopBar);
		//while (bitRead(flags, stopBar) == 0);
		delay(700);
		eStop();
		bool green = false;
		delay(1000);
		int redX = 0;
		while (!green) {
			updateBlocks();
			for (int i = 0; i < MAX_BLOCKS; i++) {
				if (detectedBlocks[i].sig() == STOP) redX = detectedBlocks[i].xPos();
			}
			for (int i = 0; i < MAX_BLOCKS; i++) {
				if (detectedBlocks[i].sig() == GREENLIGHT && abs(detectedBlocks[i].xPos() - redX) < 10) green = true;
			}
		}
		//This line should get changed to recognize the green light.
		turnCounter++;
		turnSequence();
		break;
	}
	case 1:	//Turn right at stoplight
		forward(3);
		delay(300);
		while(turn(-60));
		forward(3);
		delay(500);
		turnCounter++;
		break;
	case 2:	//Turn left to line up for ramp
		forward(2);
		delay(500);
		while (turn(70));
		turnCounter = 4;
		break;
	case 3: //Go over the ram
		//forward(3);
		//delay(2000);
		//turnCounter++;
		break;
	case 4:	//Turn left after ramp
		forward(2);
		delay(500);
		while (turn(60));
		turnCounter++;
		break;
	case 5:	//Turn right in shikane
		while (turn(-60));
		turnCounter++;
		break;
	case 6:	//Turn left to line up for Gate
		forward(2);
		delay(500);
		while (turn(60));
		turnCounter++;
		break;
	case 7:	//Turn left to approach stop sign
		forward(2);
		delay(500);
		while (turn(60));
		turnCounter++;
		break;
	case 8:	//Turn right at stop sign
		while (turn(-60));
		turnCounter++;
		break;
	case 9:	//Turn left after stop sign
		forward(2);
		delay(500);
		while (turn(60));
		turnCounter++;
		break;
	case 10:	//Go straight at second stop light
		forward(2);
		delay(1000);
		turnCounter++;
		break;
	case 11:	//Turn left toward garage
		forward(2);
		delay(500);
		while (turn(60));
		turnCounter++;
		break;
	default:
		stateChange();
		break;
	}
}

//Is there a way to create a grid using arrays where values of -1, 0 and 1 are used to indicate static obstacle, sensor decision, and marked path.  Create grid of entire course
//Instead of resetting gyro after every turn, keep a running gyro and use %360 to track heading relative to initial position, use to determine position on grid