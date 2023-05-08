#! /usr/bin/env python3

import time
from copy import deepcopy
import pandas as pd

from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from robot_nav.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

class Checkpoints(Node):

    def __init__(self):
        super().__init__('checkpoints')
        self.navigator = BasicNavigator()
        self.publisher_ = self.create_publisher(String, 'current_checkpoints', 10)
        
        self.path = '/ws_navtech/src/navtech/robot_nav/data/map/classroom/poses.csv'

    def run(self):
        # Read CSV
        df = pd.read_csv(self.path, header=None)
        
        # set of poses
        security_route = df.iloc[:, [0, 1, -2, -1]].values.tolist()

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()

        # Send our route
        while rclpy.ok():
            route_poses = []

            # Attach the poses to a "route_poses" array. Each pose must be defined as a PoseStamped message.
            for pt in security_route:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.navigator.get_clock().now().to_msg()
                pose.pose.position.x = pt[0]
                pose.pose.position.y = pt[1]
                pose.pose.orientation.z = pt[2]
                pose.pose.orientation.w = pt[3]
            
                route_poses.append(deepcopy(pose))

            # Input to call the goThroughPoses() method
            self.navigator.goThroughPoses(route_poses)

            i = 0
            while not self.navigator.isTaskComplete():

                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                            + ' seconds.')

                    # Capture the current destination
                    msg = String()
                    msg.data = '%d / %d' % (feedback.number_of_poses_remaining, len(security_route))
                    self.publisher_.publish(msg)

                    self.get_logger().info('Publisher: "%s" ' % msg.data)

                    # Some failure mode, must stop since the robot is clearly stuck
                    # If the browse task takes longer than 180 seconds, you will cancel the current task using "cancelTask()"" method
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        print('Navigation has exceeded timeout of 600s, canceling request.')
                        self.navigator.cancelTask()

            # If at end of route, reverse the route to restart
            security_route.reverse()

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Route complete! Restarting...')
            elif result == TaskResult.CANCELED:
                print('Security route was canceled, exiting.')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Security route failed! Restarting from other side...')

            else:
                print("Goal has an invalid return status!")

            self.navigator.lifecycleShutdown()
        
        exit(0)

def main(args=None):
    rclpy.init(args=args)

    checkpoints = Checkpoints().run()

    rclpy.spin(checkpoints)

    checkpoints.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
