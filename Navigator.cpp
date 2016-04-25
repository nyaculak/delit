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
    //while(!is_within(this->theta, angle, .2)) {
    while(!is_within(this->theta, angle, .2)) {
      if(is_within(this->theta, angle, .2 * 3)) {
        this->_drive_ptr->turnLeft(NAV_SLOW_TURN_SPEED);
      } else {
        this->_drive_ptr->turnLeft(NAV_TURN_SPEED);
      }
      //delay(100);
      //Serial.print("Should be turning left\th = ");
      //Serial.print(this->theta);
      //Serial.print(" Error = ");
      //Serial.println(angle - theta);
      //this->_drive_ptr->stop();
      this->update_marker();
    }
  } else {
    while(!is_within(this->theta, angle, .2)) {
      if(is_within(this->theta, angle, .2 * 3)) {
        this->_drive_ptr->turnRight(NAV_SLOW_TURN_SPEED);
      } else {
        this->_drive_ptr->turnRight(NAV_TURN_SPEED);
      }
      //delay(100);
      //Serial.println("Should be turning right");
      //this->_drive_ptr->stop();
      this->update_marker();
    }
  }
  Serial.println("Orient over");
}

void Navigator::orientZero() { //this method will make sure that the world does not explode
  this->update_marker();
  while(!is_within(this->theta, 0, .2) && !is_within(this->theta, 2*PI, .2)) {
    if(is_within(this->theta, 0, .2 * 4) || is_within(this->theta, 2*PI, .2 * 4)) {
        this->_drive_ptr->turnRight(NAV_SLOW_TURN_SPEED - .05);
    } else {
      this->_drive_ptr->turnRight(NAV_TURN_SPEED);
    }
    this->update_marker(); 
  }
}

void Navigator::translate_x(float dest) { //this method will make sure that the previous method will not explode
  this->update_marker();
  while(!this->is_within(this->x, dest, NAV_COORDINATE_TOLERANCE)) {
  //while(th
      this->_drive_ptr->drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      this->_rf_ptr->sendMessage(dest- this->x);
      this->_rf_ptr->sendMessage("\n");
      this->update_marker();
  }
}

void Navigator::translate_y(float dest) { //this method will make sure you dont divide by zero
  this->update_marker();
  while(!this->is_within(this->y, dest, NAV_COORDINATE_TOLERANCE)) {
      this->_drive_ptr->drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      this->update_marker();
  }
}

void Navigator::translate_x_angle(float dest, float angle) { //this method will make sure that the previous method will not explode
  this->update_marker();
  while(!this->is_within(this->x, dest, NAV_COORDINATE_TOLERANCE)) {
      this->drive_angle(NAV_TRANSLATE_SPEED, angle, this->theta);
      this->update_marker();
  }
}

void Navigator::translate_y_angle(float dest, float angle) { //this method will make sure you dont divide by zero
  this->update_marker();
  while(!this->is_within(this->y, dest, NAV_COORDINATE_TOLERANCE)) {
      this->drive_angle(NAV_TRANSLATE_SPEED, angle, this->theta);
      this->update_marker();
  }
}

void Navigator::drive_angle(float speed, float setpoint, float heading) {
  float error = setpoint - heading;
  float kp = speed/(PI/2);
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

float Navigator::get_angle(float x, float y, int flame_sight) { //this method will divide by zero
  // Probably need to add 2 PI since tanf returns negative angle.
  float angle = 0.0;
  
  if(flame_sight == 1) {
    angle = tanf((NAV_FLAME1_Y - y) / (NAV_FLAME1_X - x));  
  } else {
    angle = tanf((NAV_FLAME2_Y - y) / (NAV_FLAME1_X - x));
  }
  
  return angle;
}

void Navigator::turn_amount(float angle) { //this method makes sure to not start a fire
  this->update_marker();
  float target_angle = this->theta + angle;
  while(this->theta < target_angle) {
    this->update_marker();
    this->_drive_ptr->turnLeft(.4); 
  }
}
