#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped

import tf2_geometry_msgs
import tf_transformations

import tf2_ros
import tf2_py as tf
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class OdometryNode(Node):
    def __init__(self, node_name="odometry_node"):
        super().__init__(node_name=node_name)

        self.declare_parameter('encoder_resolution', 51100)
        self.declare_parameter('wheel_diameter', 0.150)
        self.declare_parameter('wheel_distance', 0.342)

        qos_profile = QoSProfile(depth=50)
        self.odom_broadcaster = StaticTransformBroadcaster(self)

        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_distance = self.get_parameter('wheel_distance').value

        self.left_ticks = 0
        self.right_ticks = 0

        self.encoder_subscription = self.create_subscription(
            Int32MultiArray,
            '/robot_base/encoders',
            self.enc_cb,
            qos_profile
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.last_time = self.get_clock().now().to_msg()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def enc_cb(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    def update(self):
        now = self.get_clock().now().to_msg()
        dt = (now.sec - self.last_time.sec) + (now.nanosec - self.last_time.nanosec) / 1e9

        # calculate wheel distances
        left_distance = self.left_ticks / self.encoder_resolution * self.wheel_diameter * math.pi
        right_distance = self.right_ticks / self.encoder_resolution * self.wheel_diameter * math.pi

        # calculate linear and angular velocity
        linear_velocity = (left_distance + right_distance) / 2.0 / dt
        angular_velocity = (right_distance - left_distance) / self.wheel_distance / dt

        # update pose
        self.theta += angular_velocity * dt
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt

        

        # publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        #quat = Quaternion()
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear = Vector3(x=float(linear_velocity), y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(angular_velocity))

        self.odom_pub.publish(odom_msg)

        # Define a transformação a ser publicada
        odom_trans = TransformStamped()
        odom_trans.header.stamp = now
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        # Create a new Quaternion message and set its fields
        odom_trans.transform.rotation.x = quat[0]
        odom_trans.transform.rotation.y = quat[1]
        odom_trans.transform.rotation.z = quat[2]
        odom_trans.transform.rotation.w = quat[3]

        self.odom_broadcaster.sendTransform(odom_trans)

        self.last_time = now

def main(args=None):
    rclpy.init(args=args)

    odometry_node = OdometryNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(odometry_node)
            odometry_node.update()
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()