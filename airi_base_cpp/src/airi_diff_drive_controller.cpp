#include "airi_base_cpp/airi_diff_drive_controller.hpp"

#include <sys/types.h>
#include <ifaddrs.h>

#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <string>

#include <ros/callback_queue.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

namespace airi {

namespace base {

namespace {

::uccn::network get_network(const std::string & interface) {
  struct ifaddrs * ifaddresses = NULL;
  if (getifaddrs(&ifaddresses) < 0) {
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
  : pnh_{"~"}, udrive_state_{"/drive/state"}, udrive_command_{"/drive/command"}
{
  std::string interface;
  if (!pnh_.getParam("interface", interface)) {
    throw std::runtime_error("No interface for uCCN was specified");
  }

  wheel_base_ = pnh_.param("wheel_base", 0.23);
  wheel_diameter_ = pnh_.param("wheel_diameter", 0.098);
  wheel_encoder_resolution_ = pnh_.param("wheel_encoder_resolution", 2048);
  odom_frame_id_ = pnh_.param<std::string>("odom_frame", "odom");
  base_frame_id_ = pnh_.param<std::string>("base_frame", "base");
  publish_tf_ = pnh_.param("publish_tf", true);

  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  left_encoder_ticks_pub_ = nh_.advertise<std_msgs::Int32>("encoders/left/ticks", 10);
  right_encoder_ticks_pub_ = nh_.advertise<std_msgs::Int32>("encoders/right/ticks", 10);

  unode_ = std::make_unique<::uccn::node>(get_network(interface), ros::this_node::getName());

  unode_->track<airi::uccn::drive_state>(
      udrive_state_, std::bind(&DiffDriveController::uDriveStateCallback,
                               this, std::placeholders::_1));

  udrive_command_provider_ = unode_->advertise(udrive_command_);

  command_sub_ = nh_.subscribe<const geometry_msgs::Twist &>(
      "cmd_vel", 10, &DiffDriveController::commandCallback, this);
}

void DiffDriveController::commandCallback(const geometry_msgs::Twist & twist) {
  airi::uccn::drive_command command;
  command.right_wheel.velocity = (
      2. * twist.linear.x + twist.angular.z * wheel_base_) / wheel_diameter_;
  command.left_wheel.velocity = (
      2. * twist.linear.x - twist.angular.z * wheel_base_) / wheel_diameter_;
  udrive_command_provider_.post(command);
}

void DiffDriveController::uDriveStateCallback(const airi::uccn::drive_state & state) {
  const ros::Time current_time = ros::Time::now();

  const double v_k = wheel_diameter_ * q16_16_to_double(
      state.right_encoder.velocity +
      state.left_encoder.velocity) / 4.;
  const double w_k = wheel_diameter_ * q16_16_to_double(
      state.right_encoder.velocity -
      state.left_encoder.velocity) / (2. * wheel_base_);

  const double dxy_k = wheel_diameter_ * q16_16_to_double(
      state.right_encoder.displacement +
      state.left_encoder.displacement) / 4.;
  const double dtheta_k = wheel_diameter_ * q16_16_to_double(
      state.right_encoder.displacement -
      state.left_encoder.displacement) / (2. * wheel_base_);

  estimated_pose_.x += dxy_k * std::cos(estimated_pose_.theta);
  estimated_pose_.y += dxy_k * std::sin(estimated_pose_.theta);
  estimated_pose_.theta += dtheta_k;

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
  odom.twist.twist.linear.x = v_k;
  odom.twist.twist.angular.z = w_k;
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

void DiffDriveController::spin()
{
  std::atomic_bool spinning{true};
  std::thread uthread([&]() {
      try {
        unode_->spin();
      } catch (const std::exception & e) {
        ROS_ERROR("%s, stopping spin\n", e.what());
      }
      spinning.store(false);
    });
  try {
    ros::CallbackQueue * callback_queue = ros::getGlobalCallbackQueue();
    while (ros::ok() && spinning.load()) {
      callback_queue->callAvailable(ros::WallDuration(0.5));
    }
  } catch (const std::exception & e) {
    ROS_ERROR("%s, stopping spin\n", e.what());
    unode_->stop();
    uthread.join();
    throw;
  }
  unode_->stop();
  uthread.join();
}

}  // namespace base

}  // namespace airi

