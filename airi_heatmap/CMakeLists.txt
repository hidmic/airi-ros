cmake_minimum_required(VERSION 2.8.3)
project(airi_heatmap)

cmake_policy(SET CMP0054 NEW)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
  scripts/scanner_node
  scripts/recorder_node
  scripts/pdf_generator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
