cmake_minimum_required(VERSION 2.8.3)
project(jet_navigation)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(stationary_controller src/stationary_controller.cpp)
target_link_libraries(stationary_controller ${catkin_LIBRARIES})
