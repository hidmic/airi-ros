#include <ros/ros.h>

#include "airi_base_cpp/airi_diff_drive_controller.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "airi_diff_drive_controller_node");

  try {
    airi::base::DiffDriveController controller;
    controller.spin();
  } catch (const std::exception & e) {
    ROS_ERROR_STREAM(e.what());
  }
}
