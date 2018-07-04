// Sonar.h

#ifndef _SONAR_h
#define _SONAR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <PWM_LIGHTS.h>

class Sonar {
public:
	//Create a combined Sonar/Servo object
	Sonar(int servoPin, int triggerPin, int echoPin, float speedOfSound, float correctionFactor);

	//Intialize the sonar object on the specified pin and home the servo.
	void init();

	//Set the servo to the specified angle
	void setAngle(int angle);

	//Get the distance to what ever is in front of the sonar (returned in CM)
	int getDistance();

	//Drive the servo to the left
	void lookLeft();

	//Drive the servo to the right
	void lookRight();

	//Cycle the servo from left to right through 5 points
	void checkServo();

	//Hardware function for checkServo()
	int lookSweep();

	void sweep(bool enable);

private:
	PWM_SERVO _tomServo;
	bool _servoSweep;
	int _servoPin;
	int _triggerPin;
	int _echoPin;
	float _speedOfSound;
	float _correctionFactor;
	float _sensorConversion;
	
};

#endif

