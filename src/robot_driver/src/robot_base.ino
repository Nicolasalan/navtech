/*--------------------------------------------------------------------

******************************************************************************
* @file robot_base.ino
* @author Isaac Jesus da Silva
* @version V3.2.0
* @created 30/03/2022
* @Modified 21/04/2023
* @e-mail isaacjesuss@gmail.com
* @revisor Isaac Jesus da Silva
* @e-mail isaacjesuss@gmail.com
****************************************************************************
/--------------------------------------------------------------------------*/

#include "Sabertooth.h"
#include "Encoder.h"
#include "NewPing.h"

#include <micro_ros_arduino.h>

#include <LiquidCrystal.h>
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
rclc_executor_t executor01;
rclc_executor_t executor02;
rcl_timer_t timer;
rcl_node_t node;
bool micro_ros_init_successful;


#define LED_PIN 25 //LED ligado na porta D25
#define LED_PIN2 24 //LED ligado na porta D24

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define WHEEL_DISTANCE 0.342        // Distance between the two wheels - Robot Slink=0.342 - Robot Woody=0.273
#define MAX_SPEED 1.0             //

#define ODOM_PERIOD 50 // 20 Hz
#define CHECK_CONNECTION_PERIOD 50 // 20Hz

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
long check_connection_timer;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
//const int rs = 44, en = 45, d4 = 49, d5 = 48, d6 = 47, d7 = 46;
LiquidCrystal lcd(44, 45, 49, 48, 47, 46);

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
}

void stopMotors() {
  STmotors.motor(1, 0);
  STmotors.motor(2, 0);
}

void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *vel = (const geometry_msgs__msg__Twist *)msgin;

  //digitalWrite(LED_PIN, (vel->linear.x <= 0) ? LOW : HIGH); 
 

  // # Extract linear and angular velocities from the message
  // #https://snapcraft.io/blog/your-first-robot-the-driver-4-5
  // #self._max_speed = rospy.get_param('~max_speed', 0.5)
  // #self._wheel_base = rospy.get_param('~wheel_base', 0.091)
  //
  // # Formulas --
  // # left_wheel_velocity  = x - tz/2
  // # right_wheel_velocity = x + tz/2
  // # x: desired linear velocity (meters per second)
  // # z: desired angular velocity (radians per second)
  // # t: track  (meters) distance between the two wheels
  //        linear = message.linear.x
  //        angular = message.angular.z
  //
  //        # Calculate wheel speeds in m/s
  //        left_speed = linear - angular*WHEEL_DISTANCE/2
  //        right_speed = linear + angular*WHEEL_DISTANCE/2
  //
  //        # Ideally we'd now use the desired wheel speeds along
  //        # with data from wheel speed sensors to come up with the
  //        # power we need to apply to the wheels, but we don't have
  //        # wheel speed sensors. Instead, we'll simply convert m/s
  //        # into percent of maximum wheel speed, which gives us a
  //        # duty cycle that we can apply to each motor.
  //        self._left_speed_percent = (100 * left_speed/self._max_speed)
  //        self._right_speed_percent = (100 * right_speed/self._max_speed)

  
  // Calculate wheel speeds in m/s
  double V1 = vel->linear.x - vel->angular.z*WHEEL_DISTANCE/2;
  double V2 = vel->linear.x + vel->angular.z*WHEEL_DISTANCE/2;

  // Verifique a conexão com o Micro-ROS
  if (state == AGENT_CONNECTED) {
    moveMotors(convertToMotor(V1), convertToMotor(V2));
  } else {
    moveMotors(convertToMotor(0), convertToMotor(0));
  }

//  moveMotors(convertToMotor(-127), convertToMotor(-127));
}

void clearEncoders() {
  Encoder1.write(0);
  Encoder2.write(0);
}

geometry_msgs__msg__Twist cmd_vel_msg;
rcl_subscription_t sub;

std_msgs__msg__Int32MultiArray enc_msg;
rcl_publisher_t pub_enc;

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

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&pub_enc, &node);
  rclc_executor_fini(&executor02);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

//============================================

void setup() {

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcd.setCursor(0, 1);
  lcd.print("waiting conn...");

  set_microros_transports();

  SabertoothTXPinSerial.begin(9600); // Serial1 (RX, TX) --> (19, 18)

  stopMotors();

  STmotors.setRamping(1);

  std_msgs__msg__Int32MultiArray__init(&enc_msg);
  enc_msg.data.size = 2;
  enc_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * enc_msg.data.size);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  digitalWrite(LED_PIN2, HIGH);  
  digitalWrite(LED_PIN, LOW);

  // Print a message to the LCD.
  lcd.setCursor(0, 1);
  lcd.print("   Conectado    ");

  clearEncoders();

  odom_timer = millis();
  check_connection_timer = millis();
}

void loop() {

  if (millis() - check_connection_timer >= CHECK_CONNECTION_PERIOD) {
      check_connection_timer = millis();

      switch (state) {
        case WAITING_AGENT:
          EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
          break;
        case AGENT_AVAILABLE:
          state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
          if (state == WAITING_AGENT) {
            destroy_entities();
          };
          break;
        case AGENT_CONNECTED:
          EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
          if (state == AGENT_CONNECTED) {
            rclc_executor_spin_some(&executor02, RCL_MS_TO_NS(0));
          }
          break;
        case AGENT_DISCONNECTED:
          destroy_entities();
          state = WAITING_AGENT;
          break;
        default:
          break;
      }
      
    // Verifique a conexão com o Micro-ROS
      if (state == AGENT_CONNECTED) {
        digitalWrite(LED_PIN2, 1);
        lcd.setCursor(0, 1);
        lcd.print("   Conectado    ");
      } else {
        digitalWrite(LED_PIN2, 0);
        lcd.setCursor(0, 1);
        lcd.print("  Desconectado  ");
        moveMotors(convertToMotor(0), convertToMotor(0));
      }
    
  }
  
  if (millis() - odom_timer >= ODOM_PERIOD) {
    enc_msg.data.data[0] = Encoder1.read();
    enc_msg.data.data[1] = Encoder2.read();

    odom_timer = millis();

    rcl_publish(&pub_enc, &enc_msg, NULL);

    clearEncoders();
    
  }

}
