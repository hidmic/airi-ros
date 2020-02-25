cmake_minimum_required(VERSION 2.8.3)
project(airi_base_cpp)

cmake_policy(SET CMP0054 NEW)

## Find catkin macros and libraries
find_package(catkin REQUIRED)

catkin_package()

#############
## Build ##
#############

add_subdirectory(uccn)



#############
## Install ##
#############
