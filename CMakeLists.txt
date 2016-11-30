cmake_minimum_required(VERSION 2.8.3)
project(common_behavior)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  diagnostic_msgs
  eigen_conversions
  kdl_conversions cmake_modules
  )

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(orocos-ocl REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file
orocos_use_package( rtt-typekit )

include_directories(${USE_OROCOS_INCLUDE_DIRS} include)

orocos_component(master_component
    src/master_component.cpp
)

target_link_libraries(master_component
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_LIBRARIES})


find_package(orocos-ocl REQUIRED)

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )

link_directories(${orocos-ocl_LIBRARY_DIRS})
include_directories(${orocos-ocl_INCLUDE_DIRS})

add_library(subsystem_deployer src/subsystem_deployer.cpp)
target_link_libraries( subsystem_deployer ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_executable( deployer src/deployer.cpp src/subsystem_deployer.cpp )
#target_link_libraries( deployer ${EXTRA_DEPS} )
target_link_libraries( deployer ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###

# Export package information (replaces catkin_package() macro) 
orocos_generate_package(
  INCLUDE_DIRS include
  LIBRARIES subsystem_deployer
  CATKIN_DEPENDS rtt_ros geometry_msgs std_msgs message_runtime rtt_roscomm common_interfaces
)


