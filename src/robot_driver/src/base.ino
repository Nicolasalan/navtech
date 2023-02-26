#include "Sabertooth.h"
#include "Encoder.h"
#include "NewPing.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>


#define WHEEL_DISTANCE 0.342        // Distance between the two wheels - Robot Slink=0.342 - Robot Woody=0.273
#define MAX_SPEED 1.0             //

#define ODOM_PERIOD 50 // 20 Hz


// Define Trig and Echo pin:
#define trigPinB 6
#define echoPinB 7
#define trigPinC 8
#define echoPinC 9
#define trigPinR 10
#define echoPinR 11
#define trigPinL 12
#define echoPinL 13

// Define maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500 cm:
#define MAX_DISTANCE 400

Sabertooth STmotors(128);

Encoder Encoder1(22, 23);
Encoder Encoder2(32, 33); //Woody

long odom_timer;

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int convertToMotor(float value) {
  float r = map(value, -MAX_SPEED, MAX_SPEED, -127.0, 127.0);
  r = constrain(r, -127.0, 127.0);
  return (int) round(r);
}

void moveMotors(int Power1, int Power2) {
  STmotors.motor(1, Power1);
  STmotors.motor(2, Power2);
}

void stopMotors() {
  STmotors.motor(1, 0);
  STmotors.motor(2, 0);
}

void cmd_vel_callback(const geometry_msgs::Twist& vel)
{
  
  // Calculate wheel speeds in m/s
  float V1 = vel.linear.x - vel.angular.z*WHEEL_DISTANCE/2;
  float V2 = vel.linear.x + vel.angular.z*WHEEL_DISTANCE/2;

  moveMotors(convertToMotor(V1), convertToMotor(V2));
//  moveMotors(convertToMotor(-127), convertToMotor(-127));
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmd_vel_callback);

void clearEncoders() {
  Encoder1.write(0);
  Encoder2.write(0);
}

std_msgs::Int32MultiArray enc_msg;
ros::Publisher pub("/robot_base/encoders", &enc_msg);

//============================================

ros::NodeHandle nh; // Serial0

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  SabertoothTXPinSerial.begin(9600); // Serial1 (RX, TX) --> (19, 18)

  stopMotors();

  STmotors.setRamping(1);

  enc_msg.data = (long *)malloc(sizeof(long) * 2); // Arduino Due -> long = int32 = 4 bytes
  enc_msg.data_length = 2;

  clearEncoders();

  odom_timer = millis();
}

void loop() {
  if (millis() - odom_timer >= ODOM_PERIOD) {
    enc_msg.data[0] = Encoder1.read();
    enc_msg.data[1] = Encoder2.read();

    odom_timer = millis();

    pub.publish(&enc_msg);

    clearEncoders();
    
  }

  nh.spinOnce();
}