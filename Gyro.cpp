// 
// 
// 

#include "Gyro.h"

Gyro::Gyro() {
	_gyro = Adafruit_L3GD20_Unified();
}

void Gyro::init() {
	_gyro.begin();
}

float Gyro::getZDot() {
	_gyro.getEvent(&_event);
	_zdot = _event.gyro.z;
	_zdot *= 180 / PI;
	if (abs(_zdot) < 0.25) _zdot = 0;
	return _zdot;
}

float Gyro::getTurned() {
	getZDot();
	_t1 = micros();
	_tt = _t1 - _t0;
	_t0 = _t1;
	_degTurned += _zdot * _tt / 1000000.0;
	return _degTurned;
}

void Gyro::resetTurned() {
	_degTurned = 0;
}
void Gyro::setTimeZero() {
	_t0 = micros();
}