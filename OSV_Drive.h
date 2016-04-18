#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#ifndef OSV_DRIVE_h
#define OSV_DRIVE_h

#define OSV_TOP_LEFT_MOTOR  1
#define OSV_BOT_LEFT_MOTOR  2
#define OSV_TOP_RIGHT_MOTOR 3
#define OSV_BOT_RIGHT_MOTOR 4

class OSV_Drive {
  public:
    OSV_Drive();
    void init();
    void drive(float, float);
    void turnLeft(float);
    void turnRight(float);
    void stop();
  private:
    Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
    Adafruit_DCMotor *topLeft = AFMS.getMotor(OSV_TOP_LEFT_MOTOR);
    Adafruit_DCMotor *botLeft = AFMS.getMotor(OSV_BOT_LEFT_MOTOR);
    Adafruit_DCMotor *topRight = AFMS.getMotor(OSV_TOP_RIGHT_MOTOR);
    Adafruit_DCMotor *botRight = AFMS.getMotor(OSV_BOT_RIGHT_MOTOR);
    
    int _map(float, float, float, float, float);
};

#endif
