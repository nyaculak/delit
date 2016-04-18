#include <Arduino.h>

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trigPin, int echoPin, int dcPin) {
  this->_trigPin = trigPin;
  this->_echoPin = echoPin;
  this->_dcPin = dcPin;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(dcPin, OUTPUT);
  digitalWrite(dcPin, HIGH);
}

int Ultrasonic::getDistance() {
  long duration;
  
  digitalWrite(this->_trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(this->_trigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(this->_trigPin, LOW);
  
  duration = pulseIn(this->_echoPin, HIGH, 70000);  
  
  if(duration <= 0) {
    digitalWrite(this->_dcPin, LOW);
    delayMicroseconds(10);
    digitalWrite(this->_dcPin, HIGH); 
  }
  
  return this->microsecondsToMillimeters(duration);
}

/*******************
 * Private methods
 ******************/
long Ultrasonic::microsecondsToMillimeters(long microseconds) {
  return microseconds / 29 / 2;
}
