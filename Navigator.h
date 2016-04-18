#include <enes100.h>
#include <enes100_rf_client.h>
#include <enes100_marker.h>
#include <SoftwareSerial.h>

#include "OSV_Drive.h"

#ifndef Navigator_h
#define Navigator_h

#define NAV_RF_DELAY_TIME 100 // ms

#define NAV_TRANSLATE_SPEED .5
#define NAV_TURN_SPEED .4

#define NAV_HEADING_TOLERANCE .1
#define NAV_COORDINATE_TOLERANCE 100

#define NAV_FLAME1_X 2700.00 //2900
#define NAV_FLAME1_Y 1250.00 //1800
#define NAV_FLAME2_X 3700.00
#define NAV_FLAME2_Y 0900.00

class Navigator {
  public:
    Navigator(enes100::Marker* marker_ptr, enes100::RfClient<SoftwareSerial>* rf_ptr, OSV_Drive* drive_ptr, int id);
    void update_marker();
    void orient(float angle);
    float get_angle(float x, float y, int flame_sight);
    void turn_amount(float angle);
    void turn_down();
    void turn_up();
    void turn_to_zero();
    float x;
    float y;
    float theta;
    boolean is_within(float value, float target, float tolerance) {
     return abs(value - target) < tolerance; 
    }
  private:
    enes100::Marker* _marker_ptr;
    enes100::RfClient<SoftwareSerial>* _rf_ptr;
    OSV_Drive* _drive_ptr;
    int _id;
};

#endif
