#include <math.h>

//Every 100ms attempt to receive marker 5 from the server.
//If marker 5 is received with an x coordinate greater than
//1 meter, report to the system that we reached the right side
//of the arena.
void loop() {
  delay(1500);
  int distance1 = analogRead(sensor);
  update_marker();
  Serial.print("Distance: "); Serial.println(distance1);
  Serial.print("Angle: "); Serial.println(get_angle(x,y, 1));
  Serial.print(x); Serial.print(", ");Serial.println(y);
    
  //rf.ReceiveMarker returns true if a marker was received
  //before timing out, and false otherwise.
}

void turn_down(){
  update_marker();
  while( theta < -PI/2 -.2) {
    update_marker();
    turn_right();
    delay(500);
    tank.turnOffMotors();
    tank.init();
  }

  while( theta < -PI/2 -.2) {
    update_marker();
    turn_left();
    delay(500);
    tank.turnOffMotors();
    tank.init();
  }
}

void turn_up() {
  update_marker();
  while( theta < PI/2 -.2) {
    update_marker();
    turn_left();
    delay(500);
    tank.turnOffMotors();
    tank.init();
  }

  while( theta < PI/2 -.2) {
    update_marker();
    turn_right();
    delay(500);
    tank.turnOffMotors();
    tank.init();
  }
}

void turn_to_zero() {
   update_marker();
  if ( theta < -.2 || theta > .2) {
    update_marker();
    turn_left();
    delay(500);
    tank.turnOffMotors();
    tank.init();
  }
}
