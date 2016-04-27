#include "FlameSensor.h"

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
