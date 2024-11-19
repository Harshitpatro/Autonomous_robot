cmake_minimum_required(VERSION 3.0.2)
project(autonomous_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)