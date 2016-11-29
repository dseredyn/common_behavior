cmake_minimum_required(VERSION 2.8.3)
project(common_behavior)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
)


