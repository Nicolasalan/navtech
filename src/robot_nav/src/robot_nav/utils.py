#!/usr/bin/env python3

# Author: Lucas Iervolino Gazignato

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped

from std_srvs.srv import Empty

class Utils():

    def __init__(self):
        self.enable_clear_costmap = rospy.get_param('~enable_clear_costmap', True)
        self.clear_costmap_period = rospy.get_param('~clear_costmap_period', 10)

        self.enable_amcl_update = rospy.get_param('~enable_amcl_update', True)
        self.amcl_updates = rospy.get_param('~amcl_updates', 10)

        self.nomotion_update = rospy.ServiceProxy('/request_nomotion_update', Empty)
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        if self.enable_amcl_update:
            rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initialpose_cb)
            rospy.loginfo('Updating AMCL pose with %d iterations...' % self.amcl_updates)

        if self.enable_clear_costmap:
            rospy.loginfo('Clearing costmap in a %d s period...' % self.clear_costmap_period)
            rate = rospy.Rate(1.0 / self.clear_costmap_period)
            while not rospy.is_shutdown():
                rate.sleep()
                try:
                    self.clear_costmap()
                    rospy.loginfo('Cleared costmap.')
                except rospy.ServiceException:
                    rospy.loginfo('move_base service not available.')

    def initialpose_cb(self, data):
        try:
            for i in range(self.amcl_updates):
                self.nomotion_update()
                rospy.sleep(0.25)
            rospy.loginfo('AMCL position updated.')
        except rospy.ServiceException:
            rospy.loginfo('AMCL service not available.')

if __name__ == '__main__':
    rospy.init_node('robot_base_utils', log_level=rospy.INFO)
    Utils()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass