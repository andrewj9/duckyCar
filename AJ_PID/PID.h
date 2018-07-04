/*
This PID control scheme was authored by Andrew S Johnson for use
in an Arduino-based autonomous robot for MEEM 4700 - Autonomous
Systems with Dr. Nina Mahmoudian.  This code authored as a class
31.March.2018
*/
#ifndef PID_h
#define PID_h

class PID {
public:
	//Create a PID controller
	PID(float pGain, float iGain, float dGain);

	//Set the maximum value for drive signal output
	void setMaxOutput(int max);

	//Calculate drive signal from error
	int getControl(int error);

private:
	float _pGain;
	float _iGain;
	float _dGain;
	unsigned long _timeLast;
	unsigned long _timeNow;
	int _errorNow;
	int _errorLast;
	int _pG;
	int _iG;
	int _dG;
	long _windup;
	int _MAXSPEED;
};

#endif