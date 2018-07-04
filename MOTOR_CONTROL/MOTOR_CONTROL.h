// MOTOR_CONTROL.h

#ifndef _MOTOR_CONTROL_h
#define _MOTOR_CONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class MOTOR_CONTROL {
public:
	//Create a motor controller object
	MOTOR_CONTROL(int pin_A1, int pin_A2, int pin_B1, int pin_B2, int pin_ENA, int pin_ENB);

	//Drive the motors with the specified drive signals (-255 to +255)
	void drive(int motA, int motB);

	//Stop all motors
	void stop();

private:
	int _pin_A1;
	int _pin_A2;
	int _pin_B1;
	int _pin_B2;
	int _pin_ENA;
	int _pin_ENB;
};

#endif

