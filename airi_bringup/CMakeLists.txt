cmake_minimum_required(VERSION 2.8.3)
project(airi_bringup)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

## Mark other files for installation (e.g. launch files, etc.)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
