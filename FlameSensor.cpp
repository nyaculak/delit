#include "FlameSensor.h"

#define MAX_ANGLE 110

FlameSensor::FlameSensor(int pin, Servo* servo) {
  this->_pin = pin;
  this->_servo = servo;
  pinMode(pin, INPUT);
}

boolean FlameSensor::detectFlame() {
  if(digitalRead(this->_pin)) { 
    return false;
  }
  return true;
}

void FlameSensor::setServo(int pos) {
  this->_servo->write(pos);
}

boolean FlameSensor::sweepForFlame() {
  int pos;
  for(pos = 0; pos < MAX_ANGLE; pos += 1) {                                  
    this->_servo->write(pos);              // tell servo to go to position in variable 'pos'
    if(this->detectFlame()) {
      return true;
    }
    delay(15);                           // waits 15ms for the servo to reach the position
  } 
  for(pos = MAX_ANGLE; pos>=0; pos-=1) {                                
    this->_servo->write(pos);              // tell servo to go to position in variable 'pos' 
    if(this->detectFlame()) {
      return true;
    }
    delay(15);                           // waits 15ms for the servo to reach the position 
  } 
  return false;
}
