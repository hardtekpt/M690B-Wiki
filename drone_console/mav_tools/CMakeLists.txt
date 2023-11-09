cmake_minimum_required(VERSION 3.0.2)
project(mav_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mavros_msgs
  sensor_msgs
  geographic_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
