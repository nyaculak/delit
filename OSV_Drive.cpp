#include "OSV_Drive.h"

OSV_Drive::OSV_Drive() {
  
}

void OSV_Drive::init() {
  this->AFMS.begin();
}

void OSV_Drive::drive(float leftVal, float rightVal) {
  leftVal = constrain(leftVal, -1, 1);
  rightVal = constrain(rightVal, -1, 1);
  
  if(leftVal >= 0) {
    this->topLeft->run(FORWARD);
    this->botLeft->run(FORWARD);
  } else {
    this->topLeft->run(BACKWARD);
    this->botLeft->run(BACKWARD);
  }
  
  if(rightVal >= 0) {
    this->topRight->run(FORWARD);
    this->botRight->run(FORWARD);
  } else {
    this->topRight->run(BACKWARD);
    this->botRight->run(BACKWARD);
  }
  
  int leftSpeed = this->_map(abs(leftVal), 0, 1, 0, 255);
  int rightSpeed = this->_map(abs(rightVal), 0, 1, 0, 255);
  
  this->topLeft->setSpeed(leftSpeed);
  this->botLeft->setSpeed(leftSpeed);
  this->topRight->setSpeed(rightSpeed);
  this->botRight->setSpeed(rightSpeed);
}

void OSV_Drive::turnLeft(float speed) {
  speed = constrain(speed, 0, 1);
  this->drive(-speed, speed);
}

void OSV_Drive::turnRight(float speed) {
  speed = constrain(speed, 0, 1);
  this->drive(speed, -speed);
}

void OSV_Drive::stop() {
  this->topLeft->run(RELEASE);
  this->botLeft->run(RELEASE);
  this->topRight->run(RELEASE);
  this->botRight->run(RELEASE);
}

/*********************
 * Private Functions *
 *********************/
int OSV_Drive::_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (int) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}


