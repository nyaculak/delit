#include <Servo.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <enes100.h>
#include <enes100_rf_client.h>
#include <enes100_marker.h>

#include "OSV_Drive.h"
#include "FlameSensor.h"
#include "Ultrasonic.h"
#include "Navigator.h"

#define SSERIAL_TX 8
#define SSERIAL_RX 9
#define FLAME_SENSOR_PIN 3
#define FLAME_SENSOR_SERVO_PIN 10

#define LEFT_ULTRASONIC_TRIG_PIN  12
#define LEFT_ULTRASONIC_ECHO_PIN  11
#define LEFT_ULTRASONIC_DC_PIN    4
#define RIGHT_ULTRASONIC_TRIG_PIN 7
#define RIGHT_ULTRASONIC_ECHO_PIN 6
#define RIGHT_ULTRASONIC_DC_PIN   5
#define FAN_PIN                   2

#define RF_ID 113

SoftwareSerial sSerial(SSERIAL_TX, SSERIAL_RX);
enes100::RfClient<SoftwareSerial> rf(&sSerial);
enes100::Marker marker;

OSV_Drive drive;
Servo flameSensorServo;
FlameSensor flameSensor(FLAME_SENSOR_PIN, &flameSensorServo);
Ultrasonic leftUltrasonic(LEFT_ULTRASONIC_TRIG_PIN, LEFT_ULTRASONIC_ECHO_PIN, LEFT_ULTRASONIC_DC_PIN);
Ultrasonic rightUltrasonic(RIGHT_ULTRASONIC_TRIG_PIN, RIGHT_ULTRASONIC_ECHO_PIN, RIGHT_ULTRASONIC_DC_PIN);
Navigator nav(&marker, &rf, &drive, RF_ID);

void setup() {
  sSerial.begin(9600);
  Serial.begin(9600);
  
  delay(500);
  
  pinMode(FAN_PIN, OUTPUT);
  drive.init();
  flameSensorServo.attach(FLAME_SENSOR_SERVO_PIN);
  rf.resetServer();
  rf.sendMessage("Team Five Connected.");
}

boolean flag = true;
void loop() {
  nav.update_marker();
  while(nav.y < NAV_FLAME1_Y) {
    nav.drive_angle(NAV_TRANSLATE_SPEED, PI/2, nav.theta);
    nav.update_marker();
  }
  twist_for_flame1();
  
  // delay(100);
}

void test_fan() {
  if(flameSensor.detectFlame()) {
    digitalWrite(FAN_PIN, HIGH); 
    delay(300);
    digitalWrite(FAN_PIN, LOW);
  } else {
    digitalWrite(FAN_PIN, LOW);
  }
  delay(10);
}

void print_pos() {  
  nav.update_marker();
  Serial.print("x: ");
  Serial.print(nav.x);
  Serial.print("\ty: ");
  Serial.print(nav.y);
  Serial.print("\th: ");
  Serial.println(nav.theta);
}

void print_dist() {
  Serial.print("L: ");
  Serial.print(leftUltrasonic.getDistance());
  Serial.print("\tR: ");
  Serial.print(rightUltrasonic.getDistance());
  Serial.print("\n");
}

void twist_for_flame1() {
  for(int pos = 60; pos < FLAME_SENSOR_MAX_ANGLE; pos += 5) {
    flameSensor.setServo(pos);
    nav.orient(PI/2 + PI/4);
    while(nav.theta > (PI/2 - PI/4)) {
      drive.turnRight(NAV_TURN_SPEED * 1.5);
      delay(100);
      drive.stop();
      delay(100);
      if(flameSensor.detectFlame()) {
        drive.stop();
        digitalWrite(FAN_PIN, HIGH);
        delay(1000);
        digitalWrite(FAN_PIN, LOW);
      }
      nav.update_marker();
    }
  }
}

void twist_for_flame2() {
  for(int pos = 60; pos < FLAME_SENSOR_MAX_ANGLE; pos += 5) {
    flameSensor.setServo(pos);
    nav.orient(PI/4);
    while(nav.theta > (PI/4)) {
      drive.turnRight(NAV_TURN_SPEED * 1.5);
      delay(100);
      drive.stop();
      delay(100);
      if(flameSensor.detectFlame()) {
        drive.stop();
        digitalWrite(FAN_PIN, HIGH);
        delay(1000);
        digitalWrite(FAN_PIN, LOW);
      }
      nav.update_marker();
    }
  }
}
    
void nav_test() {
  if(flag) {
    nav.update_marker();
    while(nav.x < NAV_FLAME1_X) {
      //drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav.theta);
      //nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav);
      nav.update_marker();
    }
    drive.stop();
    float start_angle = nav.theta;
    nav.orient(PI/2);
    while(nav.y < NAV_FLAME1_Y) {
      drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      nav.drive_angle(NAV_TRANSLATE_SPEED, PI/2, nav.theta);
      nav.update_marker();
    }
    drive.stop();
    flag = false;
  }
}

void random_nav() {
  if(flag) {
    drive_to_center();
    avoid_obstacle();
    nav_test();
    twist_for_flame1();
    flag = false;
  } 
}

void drive_to_center() {
  nav.update_marker();
  if(nav.y > 1000) {
    nav.orient(3*PI/2);
    while(nav.y > 950) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, 3*PI/2, nav.theta);
      nav.update_marker();
    }
  } else {
    nav.orient(PI/2);
    while(nav.y < 800) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, PI/2, nav.theta);
      nav.update_marker();
    }
  }
  nav.orientZero();
}

void avoid_obstacle() {
  float leftDist, rightDist;
  nav.translate_x(NAV_OBSTACLE_X - 100);
  while(nav.x < NAV_OBSTACLE_X - 100) {
    nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav.theta);
    nav.update_marker();
  }
  drive.stop();
  leftDist = leftUltrasonic.getDistance();
  rightDist = rightUltrasonic.getDistance();
  // obstacle on left side
  if(leftDist < NAV_OBSTACLE_THRESHOLD && leftDist != 0) {
    // turn left
    nav.orient(PI/2);
    // drive up
    while(nav.y < 1600) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, PI/2, nav.theta);
      nav.update_marker();
    }
    // turn right
    nav.orientZero();
    // drive forward
    while(nav.x < NAV_OBSTACLE_X + 500) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav.theta);
      nav.update_marker();
    }
    // turn "right"
    nav.orient(3*PI/2);
    // drive back to center
    while(nav.y > 1000) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, 3*PI/2, nav.theta);
      nav.update_marker();
    }
    // turn back
    nav.orientZero();
  } else if (rightDist < NAV_OBSTACLE_THRESHOLD && rightDist != 0) {
    // turn "right"
    nav.orient(3*PI/2);
    // drive down
    //nav.translate_y(500);
    while(nav.y > 400) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, 3*PI/2, nav.theta);
      nav.update_marker();
    }
    // turn back
    nav.orientZero();
    // drive infront of obstacle
    while(nav.x < NAV_OBSTACLE_X + 500) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav.theta);
      nav.update_marker();
    }
    // turn left
    nav.orient(PI/2);
    // drive back to center
    while(nav.y < 850) {
      nav.drive_angle(NAV_TRANSLATE_SPEED, PI/2, nav.theta);
      nav.update_marker();
    }
    // turn back
    nav.orientZero();
  } else {
    // do nothing
  }
}
