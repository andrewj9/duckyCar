// 
// 
// 

#include "MOTOR_CONTROL.h"

//Create a motor controller object
MOTOR_CONTROL::MOTOR_CONTROL(int pin_A1, int pin_A2, int pin_B1, int pin_B2, int pin_ENA, int pin_ENB) {
	_pin_A1 = pin_A1;
	_pin_A2 = pin_A2;
	_pin_B1 = pin_B1;
	_pin_B2 = pin_B2;
	_pin_ENA = pin_ENA;
	_pin_ENB = pin_ENB;
	pinMode(_pin_A1, OUTPUT);
	pinMode(_pin_A2, OUTPUT);
	pinMode(_pin_ENA, OUTPUT);
	pinMode(_pin_B1, OUTPUT);
	pinMode(_pin_B2, OUTPUT);
	pinMode(_pin_ENB, OUTPUT);
}

//Drive the motors with the specified drive signals (-255 to +255)
void MOTOR_CONTROL::drive(int motA, int motB) {
	if (motA < -255) motA = -255;
	if (motA > 255) motA = 255; //Throw out values that are not within the valid range
	if (motA > 0) {   //Value indicates forward movement
		digitalWrite(_pin_A1, 0);
		digitalWrite(_pin_A2, 1);
		analogWrite(_pin_ENA, motA);   //Assign Specified PWM signal to Enable A
	}
	else if (motA < 0) {  //Value indicates backward movement
		digitalWrite(_pin_A1, 1);
		digitalWrite(_pin_A2, 0);
		analogWrite(_pin_ENA, -motA);  //Assign Specified PWM signal to Enable A
	}
	else if (motA == 0) {
		digitalWrite(_pin_A1, 0);
		digitalWrite(_pin_A2, 0);
		analogWrite(_pin_ENA, 0); //Value is 0, motor A is off
	}

	if (motB < -255) motB = -255;
	if (motB > 255) motB = 255;  //Throw out values that are not within the valid range
	if (motB > 0) { //Value indicates forward movement
		digitalWrite(_pin_B1, 1);
		digitalWrite(_pin_B2, 0);
		analogWrite(_pin_ENB, motB); //Assign Specified PWM signal to Enable B
	}
	else if (motB < 0) {  //Value indicates backward movement
		digitalWrite(_pin_B1, 0);
		digitalWrite(_pin_B2, 1);
		analogWrite(_pin_ENB, -motB);  //Assign Specified PWM signal to Enable B
	}
	else if (motB == 0) {
		digitalWrite(_pin_B1, 0);
		digitalWrite(_pin_B2, 0);
		analogWrite(_pin_ENB, 0); //Value is 0, motor B is off
	}
}

//Stop all motors
void MOTOR_CONTROL::stop() {
	analogWrite(_pin_ENA, 0);
	analogWrite(_pin_ENB, 0);
}