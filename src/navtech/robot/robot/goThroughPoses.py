#! /usr/bin/env python3

import time
import pandas as pd
import numpy as np
from enum import Enum
from copy import deepcopy

from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav2_msgs.action import NavigateThroughPoses

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class Navigator(Node):

     def __init__(self):
          super().__init__('goThoughtPoses')
          self.nav_through_poses_client = ActionClient(self,
                                                            NavigateThroughPoses,
                                                            'navigate_through_poses')
          self.goal_handle = None
          self.result_future = None
          self.feedback = None
          self.status = None

          latching_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

     def goThroughPoses(self, poses, behavior_tree=''):
          """Send a `NavThroughPoses` action request."""
          self.debug("Waiting for 'NavigateThroughPoses' action server")
          while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
               self.info("'NavigateThroughPoses' action server not available, waiting...")

          goal_msg = NavigateThroughPoses.Goal()
          goal_msg.poses = poses
          goal_msg.behavior_tree = behavior_tree

          self.info(f'Navigating with {len(goal_msg.poses)} goals....')
          send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, self._feedbackCallback)

          rclpy.spin_until_future_complete(self, send_goal_future)
          self.goal_handle = send_goal_future.result()

          if not self.goal_handle.accepted:
               self.error(f'Goal with {len(poses)} poses was rejected!')
               return False

          self.result_future = self.goal_handle.get_result_async()
          return True

     def isTaskComplete(self):
          """Check if the task request of any type is complete yet."""

          if not self.result_future:
               # task was cancelled or completed
               return True
          rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
          if self.result_future.result():
               self.status = self.result_future.result().status
               if self.status != GoalStatus.STATUS_SUCCEEDED:
                    self.debug(f'Task with failed with status code: {self.status}')
                    return True
          else:
               # Timed out, still processing, not complete yet
               return False

          self.debug('Task succeeded!')
          return True

     def _feedbackCallback(self, msg):
          self.debug('Received action feedback message')
          self.feedback = msg.feedback
          return

     def getFeedback(self):
          """Get the pending action feedback message."""
          return self.feedback

     def cancelTask(self):
          """Cancel pending task request of any type."""
          self.info('Canceling current task.')
          if self.result_future:
               future = self.goal_handle.cancel_goal_async()
               rclpy.spin_until_future_complete(self, future)
          return

     def cancelNav(self):
          self.info('Canceling current goal.')
          if self.result_future:
               future = self.goal_handle.cancel_goal_async()
               rclpy.spin_until_future_complete(self, future)
          return

     def getResult(self):
          """Get the pending action result message."""
          if self.status == GoalStatus.STATUS_SUCCEEDED:
               return TaskResult.SUCCEEDED
          elif self.status == GoalStatus.STATUS_ABORTED:
               return TaskResult.FAILED
          elif self.status == GoalStatus.STATUS_CANCELED:
               return TaskResult.CANCELED
          else:
               return TaskResult.UNKNOWN

     # ============== LOGS ============== #
     def info(self, msg):
          self.get_logger().info(msg)
          return

     def warn(self, msg):
          self.get_logger().warn(msg)
          return

     def error(self, msg):
          self.get_logger().error(msg)
          return

     def debug(self, msg):
          self.get_logger().debug(msg)
          return

def main(args=None):
     rclpy.init(args=args)

     navigator = Navigator()

     path = '/ws_navtech/src/navtech/robot/config/pose.csv'

     # Read CSV
     df = pd.read_csv(path, header=None)

     # set of poses
     security_route = df.iloc[:, [0, 1, -2, -1]].values.tolist()

     route_poses = []

     while rclpy.ok():

          for pt in security_route:
               pose = PoseStamped()
               pose.header.frame_id = 'map'
               pose.header.stamp = navigator.get_clock().now().to_msg()
               pose.pose.position.x = pt[0]
               pose.pose.position.y = pt[1]
               pose.pose.orientation.z = pt[2]
               pose.pose.orientation.w = pt[3]
          
               route_poses.append(deepcopy(pose))

          navigator.goThroughPoses(route_poses)

          i = 0
          
          while not navigator.isTaskComplete():
            
               navigator.goThroughPoses(route_poses)

               i = i + 1
               feedback = navigator.getFeedback()
               if feedback and i % 5 == 0:

                    print(feedback)
            
          result = navigator.getResult()
          if result == TaskResult.SUCCEEDED:
               navigator.info('Route complete! Restarting...')
          elif result == TaskResult.CANCELED:
               navigator.warn('Security route was canceled, exiting.')
          elif result == TaskResult.FAILED:
               navigator.error('Security route failed! Restarting from other side...')

          rclpy.spin(navigator)

     exit(0)

if __name__ == '__main__':
    main()