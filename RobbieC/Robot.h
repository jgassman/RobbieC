/*
  Robot.h - Library for controlling a robot.
  Created by Jessica Thatcher, November 9, 2015.
  Released into the public domain.
*/
#ifndef Robot_h
#define Robot_h

#include "Arduino.h"

class Robot
{
  public:
    Robot();
    void speedUp();
    void forward();
	void stopMotion();
	void backward();
	void speedBack();
	void right();
	void left();
	void stayStopped();
	void sensors();
	int getFloor();
	void wander();
	
  private:
	int frontReading[3] = {0, 0, 0};
	int backReading[3] = {0, 0, 0};
	int rightReading[3] = {0, 0, 0};
	int leftReading[3] = {0, 0, 0,};
	int frontRead = 0;
	int backRead = 0;
	int leftRead = 0;
	int rightRead = 0;
	char currentMotion = ' ';
	int currentSpeed = 0;
	static int SLOW_THRESHOLD = 300;
	static int STOP_THRESHOLD = 200;
	Adafruit_BMP085_Unified bmp;
};

#endif