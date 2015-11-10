/*
  Robot.cpp - Library for controlling a robot.
  Created by Jessica Thatcher, November 9, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Robot.h"

Robot::Robot()
{
	#define rMotor 13 /**< The variable to control the right motor controller */
	#define lMotor 12 /**< The variable to control the left motor controller */

	#define frontSensor A1    /**< \def The pin connected to the front sensor */
	#define backSensor A2     /**< \def The pin connected to the back sensor */
	#define rightSensor A3    /**< \def The pin connected to the right sensor */
	#define leftSensor A4     /**< \def The pin connected to the left sensor */

	int frontReading[3] = {0, 0, 0};  /**< An array to hold readings from the front sensor */
	int backReading[3] = {0, 0, 0};   /**< An array to hold readings from the back sensor */
	int rightReading[3] = {0, 0, 0};  /**< An array to hold readings from the right sensor */
	int leftReading[3] = {0, 0, 0,};  /**< An array to hold readings from the left sensor */

	int frontRead = 0;  /**< A variable to hold the average of the last three readings from the front sensor */
	int backRead = 0;   /**< A variable to hold the average of the last three readings from the back sensor */
	int leftRead = 0;   /**< A variable to hold the average of the last three readings from the left sensor */
	int rightRead = 0;  /**< A variable to hold the average of the last three readings from the right sensor */

	char currentMotion = ' ';   /**< Keeps track of whether the robot is moving forward or backward */
	int currentSpeed = 0;       /**< Keeps track of the robot's current speed */

	static int SLOW_THRESHOLD = 300;  /**< The sensor value to signal the robot should slow down */
	static int STOP_THRESHOLD = 200;  /**< The sensor value to signal the robot to stop now */
	Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); /**< The altitude sensor */
	
	pinMode(rMotor, OUTPUT);
	pinMode(lMotor, OUTPUT);
	pinMode(frontSensor, INPUT);
	pinMode(backSensor, INPUT);
	pinMode(rightSensor, INPUT);
	pinMode(leftSensor, INPUT);
	Serial.begin(4800);
	delay(5);
	digitalWrite(rMotor, HIGH);
	digitalWrite(lMotor, HIGH);
	delayMicroseconds(1500);
	digitalWrite(rMotor, LOW);
	digitalWrite(lMotor, LOW);
	delay(20);
	Serial.begin(9600);
}

/** \brief Increases the robot's speed.
 * 
 * This function sends pulses to each motor controller so that the 
 * robot gradually speeds up to top speed. If the robot's current 
 * motion is forward, the robot will speed up in the forward direction.
 * If the robot's current direction is backward, the robot will speed
 * up in the backward direction.
 */
void Robot::speedUp() {
  if(currentMotion == 'F'){
    for(float i = currentSpeed; i <= currentSpeed + 250; i += 0.1){
      digitalWrite(rMotor, HIGH);
      digitalWrite(rMotor, HIGH);
      delayMicroseconds(i);
      digitalWrite(lMotor, LOW);
      digitalWrite(lMotor, LOW);
      delayMicroseconds(5250);
      delayMicroseconds(150);
      currentSpeed = i;
    }
  }
  else if(currentMotion == 'B'){
    for(float j = currentSpeed; j <= currentSpeed - 250; j -= 0.1){
      digitalWrite(rMotor, HIGH);
      digitalWrite(rMotor, HIGH);
      delayMicroseconds(j);
      digitalWrite(lMotor, LOW);
      digitalWrite(lMotor, LOW);
      delayMicroseconds(5250);
      delayMicroseconds(150);
      currentSpeed = j;
    }
  }
}

/** \brief Causes the robot to move forward.
 * 
 * This function sends pulses of equal width to the motors so that the
 * robot moves forward at a constant speed.
 */
void Robot::forward(){
  currentMotion = 'F';
  digitalWrite(rMotor, HIGH);
  digitalWrite(rMotor, HIGH);
  delayMicroseconds(1750);
  digitalWrite(lMotor, LOW);
  digitalWrite(lMotor, LOW);
  delayMicroseconds(5250);
}

/** \brief Causes the robot to move backward.
 *  
 * This function sends pulses of constant width to both motor 
 * controllers so that the robot will move backward at a
 * constant speed.
 */
void Robot::backward() {
  currentMotion = 'B';
  digitalWrite(rMotor, HIGH);
  digitalWrite(lMotor, HIGH);
  delayMicroseconds(1250);
  digitalWrite(rMotor, LOW);
  digitalWrite(lMotor, LOW);
  delayMicroseconds(5250);
}

** \brief Accelerates the robot backwards.
 * 
 * This function sends pulses of decreasing width to the motor
 * controllers so that the robot speeds up in the backwards direction.
 */
void Robot::speedBack(){
  currentMotion = 'B';
  for(int i = 1500; i >= 1250; i++){
    digitalWrite(rMotor, HIGH);
    digitalWrite(rMotor, HIGH);
    delayMicroseconds(i);
    digitalWrite(lMotor, LOW);
    digitalWrite(lMotor, LOW);
    delayMicroseconds(5250);
  }
}

/** \brief Stops the robot.
 * 
 * This function sends pulses of increasing width to the motor
 * controllers so that the robot slows to a stop from moving
 * backwards or sends pulses of decreasing width to the motor
 * controllers so that the robot slows to a stop from moving
 * forwards.
 */
void Robot::stopMotion(){
  if(currentMotion == 'B'){
    for(int j = 1250; j <= 1500; j++){
      digitalWrite(rMotor, HIGH);
      digitalWrite(lMotor, HIGH);
      delayMicroseconds(j);
      digitalWrite(rMotor, LOW);
      digitalWrite(lMotor, LOW);
      delayMicroseconds(5250);
      delayMicroseconds(150);
    }
  }
  else if(currentMotion == 'F'){
    for(int i = 1750; i >= 1500; i--){
      digitalWrite(rMotor, HIGH);
      digitalWrite(lMotor, HIGH);
      delayMicroseconds(j);
      digitalWrite(rMotor, LOW);
      digitalWrite(lMotor, LOW);
      delayMicroseconds(5250);
      delayMicroseconds(150);
    }
  }
  currentMotion = 'S';
}

/** \brief Causes the robot to turn right.
 *  
 * This function stops the right motor controller and sets
 * left motor controller to forward so that the robot 
 * turns left.
 */
void Robot::right() {
  for(int i = 1500; i <= 1750; i++){
    digitalWrite(lMotor, HIGH);
    delayMicroseconds(i);
    digitalWrite(lMotor, LOW);
    digitalWrite(rMotor, HIGH);
    delayMicroseconds(1500);
    digitalWrite(rMotor, LOW);
    delayMicroseconds(5250);
  }
  delay(3000);
}

/** \brief Keeps the robot from moving.
 * 
 * This function keeps the robot from moving by sending pulses of the correct width
 * to the motor controllers.
 */
void Robot::stayStopped(){
  currentMotion = 'S';
  digitalWrite(lMotor, HIGH);
  delayMicroseconds(1500);
  digitalWrite(lMotor, LOW);
  digitalWrite(rMotor, HIGH);
  delayMicroseconds(1500);
  digitalWrite(rMotor, LOW);
  delayMicroseconds(5250);
}

/** \brief Causes the robot to turn left.
 *  
 * This function stops the left motor controller and sets the
 * right motor controller to forward so that the robot 
 * turns left.
 */
void Robot::left(){
  for(int i = 1500; i<= 1750; i++){
    digitalWrite(rMotor, HIGH);
    delayMicroseconds(i);
    digitalWrite(rMotor, LOW);
    digitalWrite(lMotor, HIGH);
    delayMicroseconds(1500);
    digitalWrite(lMotor, LOW);
    delayMicroseconds(5250);
  }
  delay(3000);
}

/** \brief Reads each sensor and updates variables accordingly.
 * 
 * This function reads the value from each sensor and updates each
 * sensor's array. The oldest reading is dropped, and the newest
 * reading is added.
 */
void sensors(){
  frontReading[0] = frontReading[1];
  frontReading[1] = frontReading[2];
  frontReading[2] = analogRead(frontSensor);
  frontRead = (frontReading[0] + frontReading[1] + frontReading[2])/3;

  backReading[0] = backReading[1];
  backReading[1] = backReading[2];
  backReading[2] = analogRead(backSensor);

  rightReading[0] = rightReading[1];
  rightReading[1] = rightReading[2];
  rightReading[2] = analogRead(rightSensor);

  leftReading[0] = leftReading[1];
  leftReading[1] = leftReading[2];
  leftReading[2] = analogRead(leftSensor);
}

/** \brief Simulates random motion.
 * 
 * This function simulates random motion.
 */
void wander(){
  
}

/** \brief Reads altitude from BMP180 and determines the floor.
 *  
 *  This function retrieves the altitude from the BMP180 and uses
 *  this value to determine what floor the robot is on.
 */
int getFloor(){
  sensors_event_t event;
  bmp.getEvent(&event);
  int altitude;

  if (event.pressure){ //if the sensor read correctly
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure));
    altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
    /**< The calculated altitude from the pressure sensor */
  }
  //TODO: CREATE LOOKUP TABLE AND RETURN THE FLOOR
}