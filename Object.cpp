// 
// 
// 

#include "Object.h"


Object::Object(int signature) {
	_sig = signature;
}
void Object::update(int x, int y, int phi, int height, int width) {
	_prex2 = _prex1;
	_prey2 = _prey1;
	_prex1 = _xpos;
	_prey1 = _ypos;
	_xpos = (x + _prex1) / 2;
	_ypos = (y + _prey1) / 2;
	_phi = phi;
	_height = height;
	_width = width;
}
int Object::xPos() {
	return _xpos;
}

int Object::yPos() {
	return _ypos;
}

int Object::phi() {
	return _phi;
}

int Object::sig() {
	return _sig;
}

int Object::height() {
	return _height;
}

int Object::width() {
	return _width;
}

void Object::createNull() {
	_xpos = -1;
	_ypos = -1;
	_phi = -1;
	_height = -1;
	_width = -1;
}

String Object::toString() {
	String pt1 = "Position: (";
	String pt2 = "Angle: ";
	String pt3 = "Dimensions: ";
	
	return pt1 + '(' + _xpos + ", " + _ypos + ") " + pt2 + _phi + "° " + pt3 + _height + 'x' + _width;
}