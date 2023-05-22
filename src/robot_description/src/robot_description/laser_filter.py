#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class Filter():

    def __init__(self):
        self.pub = rospy.Publisher('/base_scan_filtered', LaserScan, queue_size=10)
        self.sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

    def laser_callback(self, data):
        filtered_scan = LaserScan()
        filtered_scan.header.seq = data.header.seq
        filtered_scan.header.stamp = data.header.stamp
        filtered_scan.header.frame_id = data.header.frame_id
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
            
            # left
            if angle >= -180 * 3.14 / 180 and angle <= -75 * 3.14 / 180:
                filtered_scan.ranges.append(data.ranges[i])
                filtered_scan.intensities.append(data.intensities[i])
            # right
            elif angle >= 75 * 3.14 / 180 and angle <= 180 * 3.14 / 180:
                filtered_scan.ranges.append(data.ranges[i])
                filtered_scan.intensities.append(data.intensities[i])
            else:
                filtered_scan.ranges.append(0)
                filtered_scan.intensities.append(0)



        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    rospy.init_node('laser_filter', log_level=rospy.INFO)
    Filter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass