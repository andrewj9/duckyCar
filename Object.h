// Object.h

#ifndef _OBJECT_h
#define _OBJECT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


class Object {
public:
	//Create an Object object for use with PixyCam
	Object(int signature);

	//Update object with new parameters
	void update(int x, int y, int phi, int height, int width);

	//Return the x-position of the object
	int xPos();

	//Return the y-position of the object
	int yPos();

	//Return the angle of the object
	int phi();

	//Return the signature of the object
	int sig();

	//Return the height of the object
	int height();

	//Return the width of the object
	int width();

	//Create a null object (all values equal to -1)
	void createNull();

	//Return data as a string
	String toString();

private:
	int _sig;
	int _xpos;
	int _ypos;
	int _phi;
	int _height;
	int _width;
	int _prex1;
	int _prey1;
	int _prex2;
	int _prey2;
};

#endif

