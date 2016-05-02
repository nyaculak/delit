#include "Navigator.h"

Navigator::Navigator(enes100::Marker* marker_ptr, enes100::RfClient<SoftwareSerial>* rf_ptr, OSV_Drive* drive_ptr, int id) {
  this->_marker_ptr = marker_ptr;
  this->_rf_ptr = rf_ptr;
  this->_drive_ptr = drive_ptr;
  this->_id = id;
}

void Navigator::update_marker() {
  delay(NAV_RF_DELAY_TIME);
  if(this->_rf_ptr->receiveMarker(this->_marker_ptr, this->_id)) {
    this->x = this->_marker_ptr->x * 1000;
    this->y = this->_marker_ptr->y * 1000;
    this->theta = (this->_marker_ptr->theta > 0) ? this->_marker_ptr->theta : this->_marker_ptr->theta + 2 * PI;
  } else {
    this->_drive_ptr->stop(); 
  }
}

void Navigator::orient(float angle) {
  this->update_marker();
  float error = angle - this->theta;
  if(is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) return;
  if(error > 0)  {
    while(!is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) {
      if(is_within(this->theta, angle, NAV_HEADING_TOLERANCE * 3)) {
        this->_drive_ptr->turnLeft(NAV_SLOW_TURN_SPEED);
      } else {
        this->_drive_ptr->turnLeft(NAV_TURN_SPEED);
      }
      this->update_marker();
    }
  } else {
    while(!is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) {
      if(is_within(this->theta, angle, NAV_HEADING_TOLERANCE * 3)) {
        this->_drive_ptr->turnRight(NAV_SLOW_TURN_SPEED);
      } else {
        this->_drive_ptr->turnRight(NAV_TURN_SPEED);
      }
      this->update_marker();
    }
  }
}

void Navigator::orientZero() {
  this->update_marker();
  while(!is_within(this->theta, 0, NAV_HEADING_TOLERANCE) && !is_within(this->theta, 2*PI, NAV_HEADING_TOLERANCE)) {
    if(is_within(this->theta, 0, NAV_HEADING_TOLERANCE * 4) || is_within(this->theta, 2*PI, NAV_HEADING_TOLERANCE * 4)) {
        this->_drive_ptr->turnRight(NAV_SLOW_TURN_SPEED - .05);
        //this->_drive_ptr->turnRight(NAV_TURN_SPEED * 1.5);
        //delay(100);
        //this->_drive_ptr->stop();
        //delay(100);
    } else {
      this->_drive_ptr->turnRight(NAV_TURN_SPEED);
    }
    this->update_marker(); 
  }
}


void Navigator::drive_angle(float speed, float setpoint, float heading) {
  float error = setpoint - heading;
  float kp = (2)*speed/(PI/2);
  if (setpoint == 0) {
    float error2 = (2*PI) - heading;
    if(abs_val(error) > abs_val(error2)) {
      error = error2;
    }
  }
  float correction = kp * error;
  correction = constrain(correction, -speed, speed);
  this->_drive_ptr->drive(speed - correction, speed + correction);
}
