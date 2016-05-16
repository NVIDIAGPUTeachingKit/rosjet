cmake_minimum_required(VERSION 2.8.3)
project(jet_driver)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jet_driver
#  CATKIN_DEPENDS controller_manager hardware_interface roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(jet_driver_node src/jet_driver_node.cpp)


target_link_libraries(jet_driver_node
   ${catkin_LIBRARIES}
)

