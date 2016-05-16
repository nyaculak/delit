#include <enes100.h>
#include <enes100_rf_client.h>
#include <enes100_marker.h>
#include <SoftwareSerial.h>

#include "OSV_Drive.h"

#ifndef Navigator_h
#define Navigator_h

#define NAV_RF_DELAY_TIME 1 // ms

#define NAV_TRANSLATE_SPEED .4
#define NAV_TURN_SPEED .4
#define NAV_SLOW_TURN_SPEED (NAV_TURN_SPEED - .05)

#define NAV_OBSTACLE_THRESHOLD 30
#define NAV_HEADING_TOLERANCE .2
#define NAV_COORDINATE_TOLERANCE 100

#define NAV_OBSTACLE_X 930.00
#define NAV_OBSTACLE_Y 100.00
#define NAV_FLAME1_X 2900.00 //2900
#define NAV_FLAME1_Y 1190.00 //1800
#define NAV_FLAME2_X 3360.00
#define NAV_FLAME2_Y 975.00

class Navigator {
  public:
    Navigator(enes100::Marker* marker_ptr, enes100::RfClient<SoftwareSerial>* rf_ptr, OSV_Drive* drive_ptr, int id);
    void update_marker();
    void orient(float angle);
    void orientZero();
    void drive_angle(float, float, float);
    float x;
    float y;
    float theta;
    boolean is_within(float value, float target, float tolerance) {
      float error = value - target;
      return (abs_val(error) < tolerance);
    }
  private:
    enes100::Marker* _marker_ptr;
    enes100::RfClient<SoftwareSerial>* _rf_ptr;
    OSV_Drive* _drive_ptr;
    int _id;
    float abs_val(float val) {
      if(val >= 0) return val;
      return -val; 
    }
};

#endif
