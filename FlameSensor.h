#include <Arduino.h>
#include <Servo.h>

#ifndef FlameSensor_h
#define FlameSensor_h

#define FLAME_SENSOR_MAX_ANGLE 110

class FlameSensor {
  public:
    FlameSensor(int, Servo*);
    boolean detectFlame();
    void setServo(int);
  private:
    int _pin;
    Servo* _servo;
};

#endif
