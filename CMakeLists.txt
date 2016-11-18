cmake_minimum_required(VERSION 2.8.3)
project(common_behavior)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  common_interfaces
  )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rtt_ros common_interfaces
)


