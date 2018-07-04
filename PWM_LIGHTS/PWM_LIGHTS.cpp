// 
// 
// 

#include "PWM_LIGHTS.h"

//Create a PWM control object for light effects
PWM_LIGHTS::PWM_LIGHTS(int en_pin, int lfi, int lfo, int lro, int lri, int rfo, int rfi, int rri, int rro) {
	_OE = en_pin;
	_LFI = lfi;
	_LFO = lfo;
	_LRO = lro;
	_LRI = lri;
	_RFO = rfo;
	_RFI = rfi;
	_RRI = rri;
	_RRO = rro;
	_pwm = Adafruit_PWMServoDriver(0x40);
	
	pinMode(_OE, OUTPUT);	
}

PWM_LIGHTS::PWM_LIGHTS(void) {
	_OE = _LFI = _LFO = _LRO = _LRI = _RFO = _RFI = _RRI = _RRO = -1;
}

PWM_SERVO::PWM_SERVO(int servo_pin) {
	_servoPin = servo_pin;
}

PWM_SERVO::PWM_SERVO(void){
	_servoPin = -1;
}


void PWM_SERVO::setServoAngle(int angle){
	int pulse = map(angle, 0, 180, 130, 450);
	_pwm.setPWM(_servoPin, 0, pulse);
}

//Initialize the pwm controller
void PWM_LIGHTS::init() {
	_pwm.begin();
}

//Specify the duty cycle in terms of cycle on/off times (uS) and the frequency (Hz)
void PWM_LIGHTS::setParams(int onTime, int offTime, int freq) {
	_ON = onTime;
	_OFF = offTime;
	_FREQ = freq;
	_pwm.setPWMFreq(_FREQ);
}

//Enable the output of the PWM controller
void PWM_LIGHTS::enable(bool enable) {
	_enabled = enable;
	digitalWrite(_OE, _enabled);
}

//Check if the output of the PWM controller is enabled
bool PWM_LIGHTS::isEnabled() {
	return _enabled;
}


//Set PWM control to drive effects based on drive signal
void PWM_LIGHTS::drive(int left, int right) {
	if (left > 0 && right < 0) {
		_pwm.setPWM(_LFI, _ON, _OFF);
		_pwm.setPWM(_LFO, _OFF, _OFF);
		_pwm.setPWM(_RFO, _ON, _OFF);
		_pwm.setPWM(_RFI, _OFF, _OFF);
		_pwm.setPWM(_LRO, _OFF, _OFF);
		_pwm.setPWM(_LRI, _ON, _OFF);
		_pwm.setPWM(_RRI, _OFF, _OFF);
		_pwm.setPWM(_RRO, _ON, _OFF);
	}
	else if (left > 0 && right > 0) {
		_pwm.setPWM(_LFI, _ON, _OFF);
		_pwm.setPWM(_LFO, _OFF, _OFF);
		_pwm.setPWM(_RFO, _OFF, _OFF);
		_pwm.setPWM(_RFI, _ON, _OFF);
		_pwm.setPWM(_LRO, _OFF, _OFF);
		_pwm.setPWM(_LRI, _ON, _OFF);
		_pwm.setPWM(_RRI, _ON, _OFF);
		_pwm.setPWM(_RRO, _OFF, _OFF);
	}
	else if (left < 0 && right > 0) {
		_pwm.setPWM(_LFI, _OFF, _OFF);
		_pwm.setPWM(_LFO, _ON, _OFF);
		_pwm.setPWM(_RFO, _OFF, _OFF);
		_pwm.setPWM(_RFI, _ON, _OFF);
		_pwm.setPWM(_LRO, _ON, _OFF);
		_pwm.setPWM(_LRI, _OFF, _OFF);
		_pwm.setPWM(_RRI, _ON, _OFF);
		_pwm.setPWM(_RRO, _OFF, _OFF);
	}
	else if (left < 0 && right < 0) {
		_pwm.setPWM(_LFI, _OFF, _OFF);
		_pwm.setPWM(_LFO, _ON, _OFF);
		_pwm.setPWM(_RFO, _ON, _OFF);
		_pwm.setPWM(_RFI, _OFF, _OFF);
		_pwm.setPWM(_LRO, _ON, _OFF);
		_pwm.setPWM(_LRI, _OFF, _OFF);
		_pwm.setPWM(_RRI, _OFF, _OFF);
		_pwm.setPWM(_RRO, _ON, _OFF);
	}
	if (left == 0) {
		_pwm.setPWM(_LFI, _OFF, _OFF);
		_pwm.setPWM(_LFO, _OFF, _OFF);
		_pwm.setPWM(_LRO, _OFF, _OFF);
		_pwm.setPWM(_LRI, _OFF, _OFF);
	}
	if (right == 0) {
		_pwm.setPWM(_RFI, _OFF, _OFF);
		_pwm.setPWM(_RFO, _OFF, _OFF);
		_pwm.setPWM(_RRO, _OFF, _OFF);
		_pwm.setPWM(_RRI, _OFF, _OFF);
	}
}

//Turn all attached LEDs on
void PWM_LIGHTS::allOn() {
	_pwm.setPWM(_LFI, _ON, _OFF);
	_pwm.setPWM(_LFO, _ON, _OFF);
	_pwm.setPWM(_RFO, _ON, _OFF);
	_pwm.setPWM(_RFI, _ON, _OFF);
	_pwm.setPWM(_LRO, _ON, _OFF);
	_pwm.setPWM(_LRI, _ON, _OFF);
	_pwm.setPWM(_RRI, _ON, _OFF);
	_pwm.setPWM(_RRO, _ON, _OFF);
}

//Turn all red LEDs on
void PWM_LIGHTS::allRed() {
	_pwm.setPWM(_RFO, _OFF, _OFF);
	_pwm.setPWM(_RFI, _ON, _OFF);
	_pwm.setPWM(_RRI, _OFF, _OFF);
	_pwm.setPWM(_RRO, _ON, _OFF);
	_pwm.setPWM(_LRO, _ON, _OFF);
	_pwm.setPWM(_LRI, _OFF, _OFF);
	_pwm.setPWM(_LFI, _ON, _OFF);
	_pwm.setPWM(_LFO, _OFF, _OFF);
}

//Turn all blue LEDs on
void PWM_LIGHTS::allBlue() {
	_pwm.setPWM(_RFO, _ON, _OFF);
	_pwm.setPWM(_RFI, _OFF, _OFF);
	_pwm.setPWM(_RRI, _ON, _OFF);
	_pwm.setPWM(_RRO, _OFF, _OFF);
	_pwm.setPWM(_LRO, _OFF, _OFF);
	_pwm.setPWM(_LRI, _ON, _OFF);
	_pwm.setPWM(_LFI, _OFF, _OFF);
	_pwm.setPWM(_LFO, _ON, _OFF);
}

//Turn all attached LEDs off
void PWM_LIGHTS::allOff() {
	_pwm.setPWM(_LFI, _OFF, _OFF);
	_pwm.setPWM(_LFO, _OFF, _OFF);
	_pwm.setPWM(_RFO, _OFF, _OFF);
	_pwm.setPWM(_RFI, _OFF, _OFF);
	_pwm.setPWM(_LRO, _OFF, _OFF);
	_pwm.setPWM(_LRI, _OFF, _OFF);
	_pwm.setPWM(_RRI, _OFF, _OFF);
	_pwm.setPWM(_RRO, _OFF, _OFF);
}

//Animate LEDs in PWM power-down sequence
void PWM_LIGHTS::offSequence() {
	allRed();
	delay(250);
	_pwm.setPWM(_LFI, _OFF, _OFF);
	_pwm.setPWM(_LFO, _OFF, _OFF);
	delay(250);
	_pwm.setPWM(_LRO, _OFF, _OFF);
	_pwm.setPWM(_LRI, _OFF, _OFF);
	delay(250);
	_pwm.setPWM(_RRI, _OFF, _OFF);
	_pwm.setPWM(_RRO, _OFF, _OFF);
	delay(250);
	_pwm.setPWM(_RFO, _OFF, _OFF);
	_pwm.setPWM(_RFI, _OFF, _OFF);
}

//Animate LEDs in PWM power-up sequence
void PWM_LIGHTS::onSequence() {
	_pwm.setPWM(_RFO, _ON, _OFF);
	_pwm.setPWM(_RFI, _OFF, _OFF);
	delay(250);
	_pwm.setPWM(_RRI, _ON, _OFF);
	_pwm.setPWM(_RRO, _OFF, _OFF);
	delay(250);
	_pwm.setPWM(_LRO, _OFF, _OFF);
	_pwm.setPWM(_LRI, _ON, _OFF);
	delay(250);
	_pwm.setPWM(_LFI, _OFF, _OFF);
	_pwm.setPWM(_LFO, _ON, _OFF);
	delay(250);
	allOff();
}