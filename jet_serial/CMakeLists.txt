cmake_minimum_required(VERSION 2.8.3)
project(jet_serial)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jet_arduino
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
