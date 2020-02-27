#include "airi_base_cpp/airi_diff_drive_controller.hpp"

#include <sys/types.h>
#include <ifaddrs.h>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <string>

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

namespace airi {

namespace base {

namespace {

::uccn::network get_network(const std::string & interface) {
  struct ifaddrs * ifaddresses = NULL;
  if (!getifaddrs(&ifaddresses)) {
    throw std::system_error(errno, std::system_category(), strerror(errno));
  }
  std::unique_ptr<struct ifaddrs, void(*)(struct ifaddrs *)> guard(ifaddresses, freeifaddrs);
  for (struct ifaddrs * it = guard.get(); it != NULL; it = it->ifa_next) {
    if (it->ifa_addr->sa_family == AF_INET) {
      if (interface == it->ifa_name) {
        return ::uccn::network(((struct sockaddr_in *)it->ifa_addr)->sin_addr,
                               ((struct sockaddr_in *)it->ifa_netmask)->sin_addr);
      }
    }
  }
  throw std::runtime_error("No '" + interface + "' found in the system");
}

}  // namespace

DiffDriveController::DiffDriveController()
  : pnh_{"~"}, udrive_state_{"/drive/state"}
{
  std::string interface;
  if (!pnh_.getParam("interface", interface)) {
    throw std::runtime_error("No interface for uCCN was specified");
  }
  unode_ = std::make_shared<::uccn::node>(get_network(interface), ros::this_node::getName());

  wheel_base_ = pnh_.param("wheel_base", 0.23);
  wheel_diameter_ = pnh_.param("wheel_diameter", 0.098);
  wheel_encoder_resolution_ = pnh_.param("wheel_encoder_resolution", 4096);
  odom_frame_id_ = pnh_.param<std::string>("odom_frame", "odom");
  base_frame_id_ = pnh_.param<std::string>("base_frame", "base");
  publish_tf_ = pnh_.param("publish_tf", true);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  left_encoder_ticks_pub_ = nh_.advertise<std_msgs::Int32>("encoders/left/ticks", 10);
  right_encoder_ticks_pub_ = nh_.advertise<std_msgs::Int32>("encoders/right/ticks", 10);

  unode_->track<airi::uccn::drive_state>(
      udrive_state_, std::bind(&DiffDriveController::uDriveStateCallback,
                               this, std::placeholders::_1));
}

void DiffDriveController::uDriveStateCallback(const airi::uccn::drive_state & state) {
  ros::Time current_time = ros::Time::now();

  double vk = 0.0, wk = 0.0;
  if (last_state_time_.isValid()) {
    airi::uccn::drive_state delta;
    delta.left_encoder.ticks = state.left_encoder.ticks - last_state_.left_encoder.ticks;
    delta.right_encoder.ticks = state.right_encoder.ticks - last_state_.right_encoder.ticks;

    vk = M_PI * wheel_diameter_ * (delta.left_encoder.ticks + delta.right_encoder.ticks) /
        (2. * wheel_encoder_resolution_);
    wk = M_PI * wheel_diameter_ * (delta.right_encoder.ticks - delta.left_encoder.ticks) /
        (wheel_base_ * wheel_encoder_resolution_);

    double dt =(current_time - last_state_time_).toSec();
    estimated_pose_.x += vk * dt * std::cos(estimated_pose_.theta);
    estimated_pose_.y += vk * dt * std::sin(estimated_pose_.theta);
    estimated_pose_.theta += wk * dt;
  }
  last_state_time_ = current_time;
  last_state_ = state;

  std_msgs::Int32 ticks;
  ticks.data = state.left_encoder.ticks;
  left_encoder_ticks_pub_.publish(ticks);
  ticks.data = state.right_encoder.ticks;
  right_encoder_ticks_pub_.publish(ticks);

  tf2::Quaternion estimated_quat;
  estimated_quat.setRPY(0., 0., estimated_pose_.theta);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = estimated_pose_.x;
  odom.pose.pose.position.y = estimated_pose_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = estimated_quat.x();
  odom.pose.pose.orientation.y = estimated_quat.y();
  odom.pose.pose.orientation.z = estimated_quat.z();
  odom.pose.pose.orientation.w = estimated_quat.w();

  odom_pub_.publish(odom);

  if (publish_tf_) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tf.transform.translation.x = estimated_pose_.x;
    tf.transform.translation.y = estimated_pose_.y;
    tf.transform.translation.z = 0.;
    tf.transform.rotation.x = estimated_quat.x();
    tf.transform.rotation.y = estimated_quat.y();
    tf.transform.rotation.z = estimated_quat.z();
    tf.transform.rotation.w = estimated_quat.w();

    tf_broadcaster_.sendTransform(tf);
  }
}

void DiffDriveController::uspin()
{
  try {
    unode_->spin();
  } catch (const std::exception & e) {
    ROS_ERROR_STREAM(e.what());
    ros::shutdown();
  }
}

void DiffDriveController::spin()
{
  std::thread uthread(std::bind(&DiffDriveController::uspin, this));

  try {
    ros::spin();
  } catch (const std::exception & e) {
    ROS_ERROR_STREAM(e.what());
    unode_->stop();
    uthread.join();
    throw;
  }
  unode_->stop();
  uthread.join();
}

}  // namespace base

}  // namespace airi

