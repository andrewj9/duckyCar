// 
// 
// 

#include "Timer.h"


//Create a new timer
Timer::Timer(uint32_t timeout) {
	_timeout = timeout;
}

//Reset the timer
void Timer::reset() {
	_setTime = millis();
}

//Set timeout length
void Timer::setTimeout(uint32_t timeout) {
	_timeout = timeout;
}

//Returns if timer has timed out
bool Timer::timeout() {
	if (millis() - _setTime > _timeout) return true;
	else return false;
}

//Returns the time since last reset
uint32_t Timer::getTime() {
	return millis() - _setTime;
}

//Returns the set time
uint32_t Timer::getSetTime() {
	return _setTime;
}