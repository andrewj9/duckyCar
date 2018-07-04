// Gyro.h

#ifndef _GYRO_h
#define _GYRO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

class Gyro {
public:
	Gyro();
	void init();
	float getZDot();
	float getTurned();
	void resetTurned();
	void setTimeZero();
private:
	float _degTurned;
	float _zdot;
	unsigned long _t0;
	unsigned long _t1;
	unsigned long _tt;
	Adafruit_L3GD20_Unified _gyro;
	sensors_event_t _event;
};

#endif

