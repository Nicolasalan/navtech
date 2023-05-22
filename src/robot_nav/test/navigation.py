#! /usr/bin/env python3

from robot_nav.topics import Mensage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from sensor_msgs.msg import JointState

import unittest
import rospy
import rostest
import time

PKG = 'robot_nav'
NAME = 'nav'

print("\033[92mNavigation Unit Tests\033[0m")

class TestROS(unittest.TestCase):

     def setUp(self):
          rospy.init_node('test_nav_node', anonymous=True) 

          self.rate = rospy.Rate(1)

          # Inicializa o nó de publicação de pose
          self.pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)
          self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
          self.client.wait_for_server()

     """Test the Map function"""
     def test_subscribe_map(self):
          # Try to receive a message from the /map topic
          msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=1.0)
          self.rate.sleep()
          # Verify that the message was received
          self.assertIsNotNone(msg, "Failed to receive message from /map topic")  

     # ======================================================================================================= #

     def test_amcl_pose(self):
          expected_pose = PoseWithCovarianceStamped()
          expected_pose.pose.pose.position.x = 0.0
          expected_pose.pose.pose.position.y = 0.0
          expected_pose.pose.pose.position.z = 0.0
          expected_pose.pose.pose.orientation.x = 0.0
          expected_pose.pose.pose.orientation.y = 0.0
          expected_pose.pose.pose.orientation.z = 0.0
          expected_pose.pose.pose.orientation.w = 1.0

          def pose_callback(data):
               self.assertAlmostEqual(data.pose.pose.position.x, expected_pose.pose.pose.position.x, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.position.y, expected_pose.pose.pose.position.y, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.position.z, expected_pose.pose.pose.position.z, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.orientation.x, expected_pose.pose.pose.orientation.x, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.orientation.y, expected_pose.pose.pose.orientation.y, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.orientation.z, expected_pose.pose.pose.orientation.z, delta=0.1)
               self.assertAlmostEqual(data.pose.pose.orientation.w, expected_pose.pose.pose.orientation.w, delta=0.1)

          rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)

          self.pose_pub.publish(expected_pose)

     """Test the move_base function"""
     def test_move_to_goal(self):
          goal = MoveBaseGoal()
          goal.target_pose.header.frame_id = "map"
          goal.target_pose.pose.position.x = 0.0
          goal.target_pose.pose.position.y = 0.0
          goal.target_pose.pose.orientation.w = 1.0

          self.client.send_goal(goal)
          self.client.wait_for_result(rospy.Duration.from_sec(40.0))

          self.assertEqual(self.client.get_state(), actionlib.GoalStatus.SUCCEEDED)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestROS)