cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
