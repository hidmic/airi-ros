cmake_minimum_required(VERSION 2.8.3)
project(airi_base_py)

cmake_policy(SET CMP0054 NEW)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS scripts/airi_base_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
