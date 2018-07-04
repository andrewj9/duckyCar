// 
// 
// 

#include "Sonar.h"

#define SERVO_WAIT 15	//milliseconds
#define SERVO_LAG 125	//milliseconds
#define SERVO_HOME 90	//degrees
#define SERVO_MAX 180	//degrees
#define SERVO_MIN 0		//degrees
#define SERVO_MID_LEFT 145	//degrees
#define SERVO_MID_RIGHT 55	//degrees
#define SERVO_CORRECTION 5 //degrees

//Create a combined Sonar/Servo object
Sonar::Sonar(int servoPin, int triggerPin, int echoPin, float soundSpeed, float sensorCorrection) {
	_servoPin = servoPin;
	_triggerPin = triggerPin;
	_echoPin = echoPin;
	_speedOfSound = soundSpeed;
	_correctionFactor = sensorCorrection;
	_servoSweep = false;
	//Ultrasonic Sensor Conversion
	//Conversion factor from http://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
	_sensorConversion = (_speedOfSound + _correctionFactor) / 2.0; // Divide by two for round-trip
	_tomServo = PWM_SERVO(_servoPin);
}

//Intialize the sonar object on the specified pin and home the servo.
void Sonar::init() {
	//Servo initialization
	//_tomServo.attach(_servoPin);
	//checkServo();
	setAngle(SERVO_HOME);

	//Ultrasonic Sensor Pin Setup
	pinMode(_triggerPin, OUTPUT);
	pinMode(_echoPin, INPUT);
}

//Set the servo to the specified angle
void Sonar::setAngle(int angle) {
	if (angle >= 0 && angle <= 180) {
		_tomServo.setServoAngle(angle + SERVO_CORRECTION);
	}
}

//Get the distance to what ever is in front of the sonar (returned in CM)
int Sonar::getDistance() {
	digitalWrite(_triggerPin, 0); //Clear the trigger pin
	//delay(20);  //Pause to let ultrasonic echoes dissipate
	digitalWrite(_triggerPin, 1); //Send a 20 uS trigger pulse
	delayMicroseconds(10);
	digitalWrite(_triggerPin, 0);
	long del = pulseIn(_echoPin, 1); //Calculate the delay for return soundwaves
	int distance = del * _sensorConversion; //Convert the time delay to a distance reading
	return distance;  //Self explanatory
}

//Drive the servo to the left
void Sonar::lookLeft() {
	setAngle(SERVO_MIN);
}

//Drive the servo to the right
void Sonar::lookRight() {
	setAngle(SERVO_MAX);
}

//Cycle the servo from left to right through 5 points
void Sonar::checkServo() {
	_servoSweep = true;
	lookSweep();
	_servoSweep = false;
}

int Sonar::lookSweep() {
	int index = 2;
	if (_servoSweep) {
		int depthField[5];
		int maxDist = 0;
		int servoAngles[] = { SERVO_MIN, 45, SERVO_HOME, 135, SERVO_MAX };
		setAngle(SERVO_MIN);
		delay(SERVO_LAG);
		for (int i = 0; i < 5; i++) {
			setAngle(servoAngles[i]);
			delay(SERVO_LAG * 2);
			depthField[i] = getDistance();
			if (depthField[i] > maxDist) {
				maxDist = depthField[i];
				index = i;
			}
		}
	}
	setAngle(SERVO_HOME);
	return index;
}

void Sonar::sweep(bool enable){
	_servoSweep = enable;
}