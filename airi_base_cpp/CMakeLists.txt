cmake_minimum_required(VERSION 3.5)
project(airi_base_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros
)

#############
## Build ##
#############

add_subdirectory(uccn)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/airi_diff_drive_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} airi-uccn)

add_executable(airi_diff_drive_controller_node src/airi_diff_drive_controller_node.cpp)

target_link_libraries(airi_diff_drive_controller_node
  ${PROJECT_NAME} ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS airi_diff_drive_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
