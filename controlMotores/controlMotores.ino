//#define USE_USBCON //Uncomment this line if you are using an arduino DUE

/*
 * Este programa recibe un número (entre -255 y 255) desde un tópico de ROS (arduino/motor_vel)
 * para controlar los motores del Rover, acorde al número recibido.
 */

/*
 * 
 * Puente H 1 es izquierda
 *      Motor 1 es trasero
 *      Motor 2 es delantero
 * Puente H 2 es derecha
 *      Motor 1 es trasero 
 *      Motor 2 es delantero
 * 
 */

 

 #include "Arduino.h"
 #include <ros.h>
 #include <std_msgs/Int32.h>

/*******************************
 * Define the material
 ******************************/
//Arduino pin definitions
#define MOT1_IN1 30  // IN1 of the L298 should be connected to this arduino pin
#define MOT1_IN2 31  // IN2 of the L298 should be connected to this arduino pin
#define MOT1_EN 3   // enable of the L2https://www.google.com/search?client=ubuntu&channel=fs&q=roboclaw+2x7A&ie=utf-8&oe=utf-898
#define MOT2_IN1 32  // IN1 of the L298 should be connected to this arduino pin
#define MOT2_IN2 33  // IN2 of the L298 should be connected to this arduino pin
#define MOT2_EN 4   // enable of the L298
#define MOT3_IN1 34  // IN1 of the L298 should be connected to this arduino pin
#define MOT3_IN2 35  // IN2 of the L298 should be connected to this arduino pin
#define MOT3_EN 5   // enable of the L2https://www.google.com/search?client=ubuntu&channel=fs&q=roboclaw+2x7A&ie=utf-8&oe=utf-898
#define MOT4_IN1 36  // IN1 of the L298 should be connected to this arduino pin
#define MOT4_IN2 37 // IN2 of the L298 should be connected to this arduino pin
#define MOT4_EN 6   // enable of the L298
#define LED 13      // A led that blinks when receiving


/*
 * Global variables
 */
ros::NodeHandle nh;
volatile int motor_vel; //The motor velocity between -255->full speed backwards and 255-> Full speed forward
volatile int motor_vel2; //The motor velocity between -255->full speed backwards and 255-> Full speed forward

/* 
 * Stop the motor
 */
void motor_stop() {
  //Stop if received an wrong direction
    digitalWrite(MOT1_IN1, 0);
    digitalWrite(MOT1_IN2, 0);
    analogWrite(MOT1_EN, 0);
    digitalWrite(MOT2_IN1, 0);
    digitalWrite(MOT2_IN2, 0);
    analogWrite(MOT2_EN, 0);
    digitalWrite(MOT3_IN1, 0);
    digitalWrite(MOT3_IN2, 0);
    analogWrite(MOT3_EN, 0);
    digitalWrite(MOT4_IN1, 0);
    digitalWrite(MOT4_IN2, 0);
    analogWrite(MOT4_EN, 0);
}

/* Moves the motor in positive sens
    vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_forward(unsigned int vel) {
    digitalWrite(MOT1_IN1, 0);
    digitalWrite(MOT1_IN2, 1);
    analogWrite(MOT1_EN, vel);
    digitalWrite(MOT2_IN1, 1);
    digitalWrite(MOT2_IN2, 0);
    analogWrite(MOT2_EN, vel);
}

void motor_forward2(unsigned int vel2) {
    digitalWrite(MOT3_IN1, 0);
    digitalWrite(MOT3_IN2, 1);
    analogWrite(MOT3_EN, vel2);
    digitalWrite(MOT4_IN1, 1);
    digitalWrite(MOT4_IN2, 0);
    analogWrite(MOT4_EN, vel2);
}

/* Moves the motor backwards
    vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_backwards(unsigned int vel) {
    digitalWrite(MOT1_IN1, 1);
    digitalWrite(MOT1_IN2, 0);
    analogWrite(MOT1_EN, vel);
    digitalWrite(MOT2_IN1, 0);
    digitalWrite(MOT2_IN2, 1);
    analogWrite(MOT2_EN, vel);
}

void motor_backwards2(unsigned int vel2) {
    digitalWrite(MOT3_IN1, 1);
    digitalWrite(MOT3_IN2, 0);
    analogWrite(MOT3_EN, vel2);
    digitalWrite(MOT4_IN1, 0);
    digitalWrite(MOT4_IN2, 1);
    analogWrite(MOT4_EN, vel2);
}


//Motor callback
void motor_vel_cb(const std_msgs::Int32 & msg) {
  digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
  motor_vel = msg.data;
  if (motor_vel > 0 && motor_vel <= 255) {
    motor_forward((unsigned int) motor_vel);
  } else if (motor_vel < 0 && motor_vel >= -255) {
    motor_backwards((unsigned int) -1*motor_vel);
  } else {
    //Stop if received a wrong value or 0
    motor_stop();
  }
}

void motor_vel_cb2(const std_msgs::Int32 & msg) {
  //digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
  motor_vel2 = msg.data;
  if (motor_vel2 > 0 && motor_vel2 <= 255) {
    motor_forward2((unsigned int) motor_vel2);
  } else if (motor_vel2 < 0 && motor_vel2 >= -255) {
    motor_backwards2((unsigned int) -1*motor_vel2);
  } else {
    //Stop if received a wrong value or 0
    motor_stop();
  }
}



//Creates the ROS subscribers
ros::Subscriber<std_msgs::Int32> motor_vel_sub("arduino/wl", motor_vel_cb);
ros::Subscriber<std_msgs::Int32> motor_vel_sub2("arduino/wr", motor_vel_cb2);
/*
 * Arduino SETUP
 */
void setup() {
Serial.begin(57600);
  
  //init ROS communication
  nh.initNode();
  //Subscribed ROS topics
  nh.subscribe(motor_vel_sub);
  nh.subscribe(motor_vel_sub2);
  // Arduino pins definition
  pinMode(LED, OUTPUT);
  pinMode(MOT1_IN1, OUTPUT);
  pinMode(MOT1_IN2, OUTPUT);
  pinMode(MOT1_EN, OUTPUT);
  pinMode(MOT2_IN1, OUTPUT);
  pinMode(MOT2_IN2, OUTPUT);
  pinMode(MOT2_EN, OUTPUT);
  pinMode(MOT3_IN1, OUTPUT);
  pinMode(MOT3_IN2, OUTPUT);
  pinMode(MOT3_EN, OUTPUT);
  pinMode(MOT4_IN1, OUTPUT);
  pinMode(MOT4_IN2, OUTPUT);
  pinMode(MOT4_EN, OUTPUT);
}

/*
 * Arduino MAIN LOOP
 */
void loop() {
  nh.spinOnce();
  delay(1);
}
