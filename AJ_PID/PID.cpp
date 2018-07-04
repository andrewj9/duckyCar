/*
This PID control scheme was authored by Andrew S Johnson for use 
in an Arduino-based autonomous robot for MEEM 4700 - Autonomous 
Systems with Dr. Nina Mahmoudian.  This code authored as a class
31.March.2018
*/
#include "PID.h"
#include <Arduino.h>

//Create a PID controller
PID::PID(float pGain, float iGain, float dGain) {
	_pGain = pGain;
	_iGain = iGain;
	_dGain = dGain;
	_timeLast = 0;
	_timeNow = 0;
	_errorNow = 0;
	_errorLast = 0;
	_pG = 0;
	_iG = 0;
	_dG = 0;
	_windup = 0;
	_MAXSPEED;
}

//Calculate drive signal from error
int PID::getControl(int error) {
		_timeLast = _timeNow;
		_timeNow = millis();
		int dt = (_timeNow - _timeLast);
		_errorLast = _errorNow;
		_errorNow = error;
		if (_errorNow == 0) _windup = 0;
		_pG = _errorNow * _pGain;
		_windup += _errorNow * dt * _iGain;
		if (_windup > 30000) _windup = 30000;
		else if (_windup < -30000) _windup = -30000;
		_iG = map(_windup, -30000, 30000, -255, 255);
		_dG = (_errorNow - _errorLast) / (dt / 1000.0) * _dGain;
		if (abs(_pG + _iG) < abs(_pG)) {
			_iG = 0;
		}
		int gain = _pG + _iG + _dG;
		gain = constrain(gain, -_MAXSPEED, _MAXSPEED);
		return gain;
}

//Set the maximum value for drive signal output
void PID::setMaxOutput(int max) {
	_MAXSPEED = max;
}