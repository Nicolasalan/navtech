#!/usr/bin/env python3

import tf
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import math

class Odom():
     WHEEL_DISTANCE = rospy.get_param('/robot_base_odom/wheel_distance')
     WHEEL_DIAMETER = rospy.get_param('/robot_base_odom/wheel_diameter')
     COUNTS_PER_REVOLUTION = rospy.get_param('/robot_base_odom/counts_per_revolution')
     DISTANCE_PER_COUNT = math.pi * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION

     def __init__(self):
          self.x = 0
          self.y = 0
          self.th = 0

          self.odom_broadcaster = tf.TransformBroadcaster()
          
          topic = rospy.get_param('~topic', '/odom')
          
          self.odom_pub = rospy.Publisher(topic, Odometry, queue_size=50)
          rospy.Subscriber('/robot_base/encoders', Int32MultiArray, self.enc_cb)

          rospy.loginfo('Computing odom data...')

     def pub(self, vx, vth):
          current_time = rospy.Time.now()
          odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

          self.odom_broadcaster.sendTransform(
               (self.x, self.y, 0.0),
               odom_quat,
               current_time,
               'base_footprint',
               'odom'
          )
          self.odom_broadcaster.sendTransform(
               (0.0, 0.0, -0.078),
               tf.transformations.quaternion_from_euler(0, 0, 0),
               current_time,
               'right_wheel',
               'right_motor'
          )
          self.odom_broadcaster.sendTransform(
               (0.0, 0.0, 0.078),
               tf.transformations.quaternion_from_euler(0, 0, 0),
               current_time,
               'left_wheel',
               'left_motor'
          )

          odom = Odometry()
          odom.header.stamp = current_time
          odom.header.frame_id = 'odom'

          odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))

          odom.child_frame_id = 'base_footprint'
          odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

          self.odom_pub.publish(odom)

     def enc_cb(self, enc_msg):
          deltaTime = 50.0 / 1000 # 50 ms (ODOM_PERIOD)

          # velocities
          v_right = enc_msg.data[1] * Odom.DISTANCE_PER_COUNT / deltaTime 
          v_left = enc_msg.data[0] * Odom.DISTANCE_PER_COUNT / deltaTime 

          # Inverse Kinematics
          v_linear = (v_right + v_left) / 2
          v_angular = (v_right - v_left) / Odom.WHEEL_DISTANCE

          # Error Correction
          if abs(v_linear) < 0.015: v_linear = 0
          if abs(v_angular) < 0.015: v_angular = 0
          
          # delta
          delta_x = (v_linear * math.cos(self.th)) * deltaTime
          delta_y = (v_linear * math.sin(self.th)) * deltaTime
          delta_th = v_angular * deltaTime

          # pose
          self.x += delta_x
          self.y += delta_y
          self.th += delta_th

          self.pub(v_linear,v_angular)

if __name__ == '__main__':
    rospy.init_node('robot_base_odom', log_level=rospy.INFO)
    Odom()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass