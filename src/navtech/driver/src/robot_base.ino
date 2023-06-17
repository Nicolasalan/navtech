#include <ESP32Encoder.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <cstring>

// msg for ROS2
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int8_multi_array.h>

rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;
rcl_node_t node;

#define WHEEL_DISTANCE_X 0.2045335 
#define WHEEL_DISTANCE_Y 0.206375 

#define MAX_SPEED 1.0           

#define ODOM_PERIOD 50 

#define MAX_DISTANCE 400

Sabertooth STmotors(128);
Sabertooth STRear(129);

// Definir os pinos dos encoders
const int encoderPin1A = 24;
const int encoderPin1B = 25;
const int encoderPin2A = 28;
const int encoderPin2B = 29;
const int encoderPin3A = 36;
const int encoderPin3B = 37;
const int encoderPin4A = 40;
const int encoderPin4B = 41;

// Definir os pinos dos motores
const int motorPin1 = 10;
const int motorPin2 = 11;
const int motorPin3 = 12;
const int motorPin4 = 13;

float V1, V2, V3, V4;

// Configurar os objetos do encoder
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

long odom_timer;

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int convertToMotor(double value) {
  double r = map(value, -MAX_SPEED, MAX_SPEED, -127.0, 127.0);
  r = constrain(r, -127.0, 127.0);
  return (int) round(r);
}

void moveMotors(int Power1, int Power2) {
  STmotors.motor(1, Power1);
  STmotors.motor(2, Power2);
  STRear.motor(1, Power3);
  STRear.motor(2, Power4);
}

void stopMotors() {
  STmotors.motor(1, 0);
  STmotors.motor(2, 0);
  STRear.motor(1, 0);
  STRear.motor(2, 0);
}

void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *vel = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate wheel speeds in m/s {Mecanum Kinematics}
  V1 = vel.linear.x - vel.linear.y - (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V2 = vel.linear.x + vel.linear.y + (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V3 = vel.linear.x + vel.linear.y - (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
  V4 = vel.linear.x - vel.linear.y + (WHEEL_DISTANCE_X + WHEEL_DISTANCE_Y) * vel.angular.z;
}

void clearEncoders() {
  Encoder1.write(0);
  Encoder2.write(0);
  Encoder3.write(0);
  Encoder4.write(0);
}

geometry_msgs__msg__Twist cmd_vel_msg;
rcl_subscription_t sub;

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "robot_base", "", &support));

  //---cmd_vel dos motores----------------------------------
  RCCHECK(rclc_subscription_init_default(
  &sub,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  "/cmd_vel"));

  executor02 = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor02, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor02, &sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  //---------------------------------------------------------

  //---Encoders----------------------------------------------
  RCCHECK(rclc_publisher_init_default(
    &pub_enc,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/robot_base/encoders"));
  //---------------------------------------------------------

  return true;
}

//============================================

void setup() {

  set_microros_transports();

  SabertoothTXPinSerial.begin(9600); // Serial1 (RX, TX) --> (19, 18)

  stopMotors();

  STmotors.setRamping(1);

  clearEncoders();

  odom_timer = millis();
}

void loop() {
  
  if (millis() - odom_timer >= ODOM_PERIOD) {
    enc_msg.data.data[0] = Encoder1.read();
    enc_msg.data.data[1] = Encoder2.read();
    enc_msg.data.data[2] = Encoder2.read();
    enc_msg.data.data[3] = Encoder3.read();

    odom_timer = millis();

    rcl_publish(&pub_enc, &enc_msg, NULL);

    clearEncoders();
    
  }
}