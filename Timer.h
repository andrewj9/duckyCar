// Timer.h

#ifndef _TIMER_h
#define _TIMER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif
class Timer {
public:
	//Create a new millisecond timer
	Timer(uint32_t timeout);
	//Reset the timer
	void reset();
	//Set timeout length
	void setTimeout(uint32_t timeout);
	//Returns if timer has timed out
	bool timeout();
	//Returns the time since last reset
	uint32_t getTime();
	//Returns the set time
	uint32_t getSetTime();

private:
	uint16_t _timeout;
	uint32_t _setTime;
};

#endif

