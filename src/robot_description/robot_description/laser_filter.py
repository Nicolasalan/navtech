#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class Filter(Node):

    def __init__(self):
        super().__init__('Filter')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.pub = self.create_publisher(LaserScan, '/base_scan_front_filtered', 10)
        self.sub = self.create_subscription(LaserScan, '/base_scan_front', self.laser_callback, qos_profile)

    def laser_callback(self, data):
        filtered_scan = LaserScan()
        filtered_scan.header = data.header
        filtered_scan.angle_min = data.angle_min
        filtered_scan.angle_max = data.angle_max
        filtered_scan.angle_increment = data.angle_increment
        filtered_scan.time_increment = data.time_increment
        filtered_scan.scan_time = data.scan_time
        filtered_scan.range_min = data.range_min
        filtered_scan.range_max = data.range_max
        filtered_scan.ranges = []
        filtered_scan.intensities = []

        for i in range(len(data.ranges)):
            angle = data.angle_min + i * data.angle_increment

            # front left
            if angle >= -180 * 3.14 / 180 and angle <= -75 * 3.14 / 180:
                filtered_scan.ranges.append(data.ranges[i])
                filtered_scan.intensities.append(data.intensities[i])
            # front right
            elif angle >= 75 * 3.14 / 180 and angle <= 180 * 3.14 / 180:
                filtered_scan.ranges.append(data.ranges[i])
                filtered_scan.intensities.append(data.intensities[i])
            # ignored
            else:
                filtered_scan.ranges.append(0)
                filtered_scan.intensities.append(0)

        self.pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    filter_node = Filter()
    rclpy.spin(filter_node)
    filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
