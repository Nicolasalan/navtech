cmake_minimum_required(VERSION 3.0.2)
project(robot_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf
)

add_service_files(
  DIRECTORY 
  srv
  FILES
  FollowWaypointsService.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)