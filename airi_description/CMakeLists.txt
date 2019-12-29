cmake_minimum_required(VERSION 2.8.3)
project(airi_description)

cmake_policy(SET CMP0054 NEW)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  pcl_conversions
  roslaunch
  rviz
  sensor_msgs
  xacro
)

catkin_package(
  LIBRARIES airi_bumper_plugin
)

#############
## Build ##
#############

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_library(airi_bumper_plugin plugins/airi_bumper_plugin.cpp)
target_link_libraries(airi_bumper_plugin ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS airi_bumper_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config urdf media launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
