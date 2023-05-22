#! /usr/bin/env python3

import unittest
import rosunit

PKG = 'robot_nav'
NAME = 'library'

print("\033[92mLibrary Unit Tests\033[0m")

class TestPackage(unittest.TestCase):

     def setUp(self):
          pass 

     def test_rospy_import(self):
          try:
               import rospy
          except ImportError:
               self.fail("Could not import rospy")

     def test_geometry_msgs_import(self):
          try:
               import geometry_msgs
          except ImportError:
               self.fail("Could not import geometry_msgs")

     def test_nav_msgs_import(self):
          try:
               import nav_msgs
          except ImportError:
               self.fail("Could not import nav_msgs")

     def test_move_base_msgs_import(self):
          try:
               import move_base_msgs
          except ImportError:
               self.fail("Could not import move_base_msgs")

     def test_std_msgs_import(self):
          try:
               import std_msgs
          except ImportError:
               self.fail("Could not import std_msgs")

     def test_dynamic_reconfigure_import(self):
          try:
               import dynamic_reconfigure
          except ImportError:
               self.fail("Could not import dynamic_reconfigure")

     def test_dynamic_reconfigure_import(self):
          try:
               import dynamic_reconfigure
          except ImportError:
               self.fail("Could not import dynamic_reconfigure")

     def test_rospkg_import(self):
          try:
               import rospkg
          except ImportError:
               self.fail("Could not import rospkg")

     def test_os_import(self):
          try:
               import os
          except ImportError:
               self.fail("Could not import os")

     def test_os_import(self):
          try:
               import os
          except ImportError:
               self.fail("Could not import os")

     def test_collections_import(self):
          try:
               import collections
          except ImportError:
               self.fail("Could not import collections")

     def test_time_import(self):
          try:
               import time
          except ImportError:
               self.fail("Could not import time")

     def test_actionlib_import(self):
          try:
               import actionlib
          except ImportError:
               self.fail("Could not import actionlib")

     def test_csv_import(self):
          try:
               import csv
          except ImportError:
               self.fail("Could not import csv")

     def test_tf_import(self):
          try:
               import tf
          except ImportError:
               self.fail("Could not import tf")

     def test_actionlib_msgs_import(self):
          try:
               import actionlib_msgs
          except ImportError:
               self.fail("Could not import actionlib_msgs")

     def test_std_srvs_import(self):
          try:
               import std_srvs
          except ImportError:
               self.fail("Could not import std_srvs")

     def test_robot_nav_import(self):
          try:
               import robot_nav
          except ImportError:
               self.fail("Could not import robot_nav")

     def test_datetime_import(self):
          try:
               import datetime
          except ImportError:
               self.fail("Could not import datetime")

     def test_shutil_import(self):
          try:
               import shutil
          except ImportError:
               self.fail("Could not import shutil")

     def test_math_import(self):
          try:
               import math
          except ImportError:
               self.fail("Could not import math")

     def test_sensor_msgs_import(self):
          try:
               import sensor_msgs
          except ImportError:
               self.fail("Could not import sensor_msgs")

     def test_gazebo_msgs_import(self):
          try:
               import gazebo_msgs
          except ImportError:
               self.fail("Could not import gazebo_msgs")

     def test_trimesh_import(self):
          try:
               import trimesh
          except ImportError:
               self.fail("Could not import trimesh")

     def test_scipy_import(self):
          try:
               import scipy
          except ImportError:
               self.fail("Could not import scipy")

if __name__ == '__main__':
    rosunit.unitrun(PKG, NAME, TestPackage)