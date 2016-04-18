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
  }
}

void Navigator::orient(float angle) {
  this->update_marker();
  float error = angle - this->theta;
  if(is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) return;
  if(error > 0)  {
    while(!is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) {
      this->_drive_ptr->turnLeft(NAV_TURN_SPEED);
      this->update_marker();
      //Serial.print("Heading: ");
      //Serial.println(this->theta);
    }
  } else {
    while(!is_within(this->theta, angle, NAV_HEADING_TOLERANCE)) {
      this->_drive_ptr->turnRight(NAV_TURN_SPEED);
      this->update_marker();
    }
  }
}

float Navigator::get_angle(float x, float y, int flame_sight) {
  // Probably need to add 2 PI since tanf returns negative angle.
  float angle = 0.0;
  
  if(flame_sight == 1) {
    angle = tanf((NAV_FLAME1_Y - y) / (NAV_FLAME1_X - x));  
  } else {
    angle = tanf((NAV_FLAME2_Y - y) / (NAV_FLAME1_X - x));
  }
  
  return angle;
}

void Navigator::turn_amount(float angle) {
  this->update_marker();
  float target_angle = this->theta + angle;
  while(this->theta < target_angle) {
    this->update_marker();
    this->_drive_ptr->turnLeft(.4); 
  }
}
