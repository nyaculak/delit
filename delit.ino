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
#define LEFT_IR_SENSOR_PIN A0
#define RIGHT_IR_SENSOR_PIN A1

#define LEFT_ULTRASONIC_TRIG_PIN  12
#define LEFT_ULTRASONIC_ECHO_PIN  11
#define LEFT_ULTRASONIC_DC_PIN    4
#define RIGHT_ULTRASONIC_TRIG_PIN 7
#define RIGHT_ULTRASONIC_ECHO_PIN 6
#define RIGHT_ULTRASONIC_DC_PIN   5
#define FAN_PIN 2

#define RF_ID 112

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
  //drive.drive(.6, .6);
  
  //turn_test();
  //nav_test();
  //drive_to_center();
  random_nav();
  //print_pos();
  /*
  if(flag) {
    nav.translate_x(NAV_FLAME1_X);
    flag = false;
  }
  */
  //print_dist();
  //test_fan();
  
  //print_pos();
  // flame_test();
  // delay(100);
}

void test_fan() {
  if(flameSensor.sweepForFlame()) {
    digitalWrite(FAN_PIN, HIGH); 
    delay(300);
    digitalWrite(FAN_PIN, LOW);
  } else {
    digitalWrite(FAN_PIN, LOW);
  }
  delay(10);
}

void print_dist() {
  Serial.print("L: ");
  Serial.print(leftUltrasonic.getDistance());
  Serial.print("\tR: ");
  Serial.print(rightUltrasonic.getDistance());
  Serial.print("\n");
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

void flame_test() {
  if(flameSensor.sweepForFlame()) {
    Serial.println("FLAME");
  } else {
    Serial.println("NO FLAME");
  }
}

void turn_test() {
  //static flag;
  delay(100);
  if(flag) {
    //Serial.println("Begin Turn Test");
    rf.sendMessage("Begin Turn Test");
    nav.update_marker();
    float start_angle = nav.theta;
    //Serial.println("Turning to PI/2");
    rf.sendMessage("Turning to PI/2");
    nav.orient(start_angle + PI/2);
    drive.stop();
    delay(500);
    //Serial.println("Turning to PI");
    rf.sendMessage("Turning to PI");
    nav.orient(start_angle + PI);
    drive.stop();
    delay(500);
    //Serial.println("Turning to 3PI/2");
    rf.sendMessage("Turning to 3PI/2");
    nav.orient(start_angle + 3*PI/2);
    drive.stop();
    flag = false;
    //Serial.println("Turning Done");
    rf.sendMessage("Turning Done");
  }
}

void nav_test() {
  if(flag) {
    nav.update_marker();
    while(nav.x < NAV_FLAME1_X) {
      drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      //nav.drive_angle(NAV_TRANSLATE_SPEED, 0, nav);
      nav.update_marker();
    }
    drive.stop();
    float start_angle = nav.theta;
    nav.orient(PI/2);
    while(nav.y < NAV_FLAME1_Y) {
      drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      nav.update_marker();
    }
    drive.stop();
    flag = false;
  }
}

void random_nav() {
  if(flag) {
    drive_to_center();
    //avoid_obstacle();
    nav_test();
    flag = false;
  } 
}

void drive_to_center() {
  nav.update_marker();
  if(nav.y > 1000) {
    nav.orient(3*PI/2);
    //while(!nav.is_within(nav.y, 1050, NAV_COORDINATE_TOLERANCE)) {
    while(nav.y > 1050) {
      drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      nav.update_marker();
    }
  } else {
    nav.orient(PI/2);
    //while(!nav.is_within(nav.y, 900, NAV_COORDINATE_TOLERANCE)) {
    while(nav.y < 900) {
      drive.drive(NAV_TRANSLATE_SPEED, NAV_TRANSLATE_SPEED);
      nav.update_marker();
    }
  }
  nav.orientZero();
}

void avoid_obstacle() {
  float leftDist, rightDist;
  nav.translate_x(NAV_OBSTACLE_X - 100);
  drive.stop();
  leftDist = leftUltrasonic.getDistance();
  rightDist = rightUltrasonic.getDistance();
  // obstacle on left side
  if(leftDist < NAV_OBSTACLE_THRESHOLD) {
    // turn left
    nav.orient(PI/2);
    // drive up
    nav.translate_y(1500);
    // turn right
    nav.orientZero();
    // drive forward
    nav.translate_x(NAV_OBSTACLE_X + 500);
    // turn "right"
    nav.orient(3*PI/2);
    // drive back to center
    nav.translate_y(1000);
    // turn back
    nav.orientZero();
  } else if (rightDist < NAV_OBSTACLE_THRESHOLD) {
    // turn "right"
    nav.orient(3*PI/2);
    // drive down
    nav.translate_y(500);
    // turn back
    nav.orientZero();
    // drive infront of obstacle
    nav.translate_x(NAV_OBSTACLE_X + 500);
    // turn left
    nav.orient(PI/2);
    // drive back to center
    nav.translate_y(1000);
    // turn back
    nav.orientZero();
  } else {
    // do nothing
  }
}
