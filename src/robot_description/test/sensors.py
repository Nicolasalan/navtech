#! /usr/bin/env python3

from robot_nav.topics import Mensage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import JointState

import unittest
import rospy
import rostest
import time

PKG = 'robot_nav'
NAME = 'sensors'

print("\033[92mSensors Unit Tests\033[0m")

class TestROS(unittest.TestCase):

     def setUp(self):
          rospy.init_node('test_sensors_node', anonymous=True) 

          self.rc = Mensage()
          self.success = False
          self.rate = rospy.Rate(1)

     def callback(self, msg):
          self.success = msg.angular.z and msg.angular.z == 1
     
     # ======================================================================================================= #

     """Test the velocity function"""
     def test_publish_cmd_vel(self):
          # Test function for the publish_cmd_vel function.   
          test_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
          self.rc.cmd_vel.angular.z = 1
          self.rc.publish_cmd_vel()
          timeout_t = time.time() + 1.0  # 10 seconds
          while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
               time.sleep(0.1)
          self.assert_(self.success)
     
     # ======================================================================================================= #

     """Test the odom function"""
     def test_subscribe_odom(self):
          # Try to receive a message from the /odom topic
          msg = rospy.wait_for_message("odom", Odometry, timeout=1.0)
          self.rate.sleep()
          # Verify that the message was received
          self.assertIsNotNone(msg, "Failed to receive message from /odom topic")
     
     # ======================================================================================================= #

     """Test the scan function"""
     def test_subscribe_scan(self):
          # Try to receive a message from the /base_scan topic
          msg = rospy.wait_for_message("base_scan", LaserScan, timeout=1.0)
          self.rate.sleep()
          # Verify that the message was received
          self.assertIsNotNone(msg, "Failed to receive message from /base_scan topic")

     # ======================================================================================================= #   
     
     """Test the joint publishing function"""
     def test_joint_publishing(self):
          self.assertTrue(rospy.wait_for_message('/joint_states', JointState, timeout=1))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestROS)