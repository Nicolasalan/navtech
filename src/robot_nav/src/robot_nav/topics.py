#! /usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy

class Mensage():
     def __init__(self):       

          self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
          self.cmd_vel = Twist()
          self.ctrl_c = False
          self.rate = rospy.Rate(1)

     def publish_cmd_vel(self): 
          """Publishes a command velocity message to control the robot's movement."""

          while not self.ctrl_c:
               connections = self.vel_publisher.get_num_connections()
               if connections > 0:
                    self.vel_publisher.publish(self.cmd_vel)
                    break
               else:
                    self.rate.sleep()
                    
     def shutdownhook(self):
          """Shutdown hook for the node."""

          rospy.is_shutdown()