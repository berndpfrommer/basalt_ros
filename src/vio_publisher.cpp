// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "basalt_ros/vio_publisher.hpp"

#include <fstream>
#include <ios>
#include <stdexcept>

static geometry_msgs::msg::Point to_ros_point(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Point vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Vector3 to_ros_vec(const Eigen::Vector3d & v)
{
  geometry_msgs::msg::Vector3 vv;
  vv.x = v[0];
  vv.y = v[1];
  vv.z = v[2];
  return (vv);
}

static geometry_msgs::msg::Quaternion to_ros_quat(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion qq;
  qq.x = q.x();
  qq.y = q.y();
  qq.z = q.z();
  qq.w = q.w();
  return (qq);
}

static geometry_msgs::msg::TransformStamped to_tf_msg(
  const rclcpp::Time & t, const Eigen::Quaterniond & q,
  const Eigen::Vector3d & trans, const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = t;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;

  tf.transform.translation = to_ros_vec(trans);
  tf.transform.rotation = to_ros_quat(q);
  return (tf);
}

namespace basalt_ros
{
VIOPublisher::VIOPublisher(rclcpp::Node * node) : node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "starting publisher");

  tfBroadCaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  // we don't do covariance quite yet....
  cov_ = {0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
          0.00, 0.00, 0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00,
          0.00, 0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00,
          0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.01};

  node_->get_parameter_or<std::string>(
    "world_frame_id", msg_.header.frame_id, "world");
  node_->get_parameter_or<std::string>(
    "odom_frame_id", msg_.child_frame_id, "odom");

  std::vector<double> ext_trans = {0, 0, 0};
  std::vector<double> ext_q = {1.0, 0, 0, 0};

  if (!node_->has_parameter("extra_translation")) {
    ext_trans = node_->declare_parameter("extra_translation", ext_trans);
  }
  if (node_->get_parameter<std::vector<double>>(
        "extra_translation", ext_trans)) {
    if (ext_trans.size() != 3) {
      RCLCPP_ERROR(
        node->get_logger(), "extra_translation must have 3 elements!");
      throw std::invalid_argument("extra translation must have 3 elements");
    }
  }
  // rotation is given in format (w, x, y, z)
  if (!node_->has_parameter("extra_rotation")) {
    ext_q = node_->declare_parameter("extra_rotation", ext_q);
  }
  if (node_->get_parameter<std::vector<double>>("extra_rotation", ext_q)) {
    if (ext_q.size() != 4) {
      RCLCPP_ERROR(node->get_logger(), "extra_rotation must have 4 elements!");
      throw std::invalid_argument("extra rotation must have 4 elements");
    }
  }

  T_extra_ = Eigen::Vector3d(ext_trans[0], ext_trans[1], ext_trans[2]);
  q_extra_ = Eigen::Quaterniond(ext_q[0], ext_q[1], ext_q[2], ext_q[3]);
  extraTF_ = T_extra_.norm() > 0.001 || q_extra_.w() < 0.99999;
  if (extraTF_) {
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "extra transform: q=" << q_extra_.coeffs().transpose()
                            << ", T=" << T_extra_.transpose());
  } else {
    RCLCPP_INFO(node->get_logger(), "no extra transform specified");
  }
}

void VIOPublisher::publish(const BasaltPoseVelBiasState::Ptr & data)
{
  const Eigen::Vector3d T_orig = data->T_w_i.translation();
  const Eigen::Quaterniond q_orig = data->T_w_i.unit_quaternion();
  const Eigen::Vector3d ang_vel = data->vel_w_i;
  const Eigen::Vector3d T = extraTF_ ? (q_extra_ * T_orig + T_extra_) : T_orig;
  const Eigen::Quaterniond q = extraTF_ ? (q_extra_ * q_orig) : q_orig;

  // make odometry message
  msg_.header.stamp.sec = data->t_ns / 1000000000LL;
  msg_.header.stamp.nanosec = data->t_ns % 1000000000LL;

  msg_.pose.pose.position = to_ros_point(T);
  msg_.pose.pose.orientation = to_ros_quat(q);

  msg_.pose.covariance = cov_;
  msg_.twist.twist.linear = to_ros_vec(ang_vel);
  msg_.twist.covariance = cov_;  // zero matrix

  pub_->publish(msg_);
#if 0
    std::cout << "position: " << msg_.pose.pose.position.x << " "
              << msg_.pose.pose.position.y << " " << msg_.pose.pose.position.z
              << std::endl;
#endif
  // make transform message
  const rclcpp::Time t(msg_.header.stamp.sec, msg_.header.stamp.nanosec);
  const geometry_msgs::msg::TransformStamped tf =
    to_tf_msg(t, q, T, msg_.header.frame_id, msg_.child_frame_id);

  tfBroadCaster_->sendTransform(tf);
}

}  // namespace basalt_ros
