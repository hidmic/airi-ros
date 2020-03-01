#include <ros/ros.h>

#include "airi_base_cpp/airi_diff_drive_controller.hpp"

int main(int argc, char *argv[]) {
  int ret = 0;

  ros::init(argc, argv, "airi_diff_drive_controller_node");

  try {
    airi::base::DiffDriveController controller;
    controller.spin();
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    ret = -1;
  }

  ros::shutdown();
  return ret;
}
