#ifndef AIRI_BASE_CPP_AIRI_BASE_CONTROLLER_HPP_
#define AIRI_BASE_CPP_AIRI_BASE_CONTROLLER_HPP_

#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include "airi/uccn.hpp"

namespace airi {

namespace base {

class DiffDriveController {
public:
  DiffDriveController();

  void spin();

private:
  // ROS callbacks
  void commandCallback(const geometry_msgs::Twist & twist);

  // uCCN callbacks
  void uDriveStateCallback(const airi::uccn::drive_state & state);

  // ROS interfaces
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher odom_pub_;
  ros::Subscriber command_sub_;
  ros::Publisher left_encoder_ticks_pub_;
  ros::Publisher right_encoder_ticks_pub_;
  ros::Publisher left_encoder_vel_pub_;
  ros::Publisher right_encoder_vel_pub_;
  ros::Publisher left_encoder_pos_pub_;
  ros::Publisher right_encoder_pos_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // uCCN interfaces
  std::unique_ptr<::uccn::node> unode_;
  ::uccn::record<airi::uccn::drive_state> udrive_state_;
  ::uccn::record<airi::uccn::drive_command> udrive_command_;
  ::uccn::record_provider<airi::uccn::drive_command> udrive_command_provider_;

  // Controller state
  double wheel_base_;
  double wheel_diameter_;
  double wheel_encoder_resolution_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_tf_;

  geometry_msgs::Pose2D estimated_pose_;
  airi::uccn::drive_state last_state_;
  ros::Time last_state_time_;
};

}  // namespace base

}  // namespace airi

#endif  // AIRI_BASE_CPP_AIRI_BASE_CONTROLLER_HPP_
