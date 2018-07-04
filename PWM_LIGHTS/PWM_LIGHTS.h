// PWM_LIGHTS.h

#ifndef _PWM_LIGHTS_h
#define _PWM_LIGHTS_h
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class PWM_LIGHTS {
public:
	//Create a PWM control object for light effects
	PWM_LIGHTS(int en_pin, int lfi, int lfo, int lro, int lri, int rfo, int rfi, int rri, int rro);
	PWM_LIGHTS(void);

	//Initialize the pwm controller
	void init();

	//Specify the duty cycle in terms of cycle on/off times (uS) and the frequency (Hz)
	void setParams(int onTime, int offTime, int freq);

	//Enable the output of the PWM controller
	void enable(bool enable);

	//Check if the output of the PWM controller is enabled
	bool isEnabled();

	//Set PWM control to drive effects based on drive signal
	void drive(int left, int right);

	//Animate LEDs in PWM power-up sequence
	void onSequence();

	//Animate LEDs in PWM power-down sequence
	void offSequence();

	//Turn all attached LEDs on
	void allOn();

	//Turn all red LEDs on
	void allRed();

	//Turn all blue LEDs on
	void allBlue();

	//Turn all attached LEDs off
	void allOff();

	Adafruit_PWMServoDriver _pwm;

private:
	int _OE;
	int _LFI;
	int _LFO;
	int _LRO;
	int _LRI;
	int _RFO;
	int _RFI;
	int _RRI;
	int _RRO;
	int _ON;
	int _OFF;
	int _FREQ;
	bool _enabled;	
};

class PWM_SERVO : public PWM_LIGHTS {
	public:
			//Create a PWM control object for the servo
	PWM_SERVO(int servo_pin);
	PWM_SERVO(void);

		//Control the Servo
	void setServoAngle(int angle);

private:
	int _servoPin;
};

#endif

