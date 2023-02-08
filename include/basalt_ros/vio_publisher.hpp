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

#ifndef BASALT_ROS__VIO_PUBLISHER_HPP_
#define BASALT_ROS__VIO_PUBLISHER_HPP_
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "basalt_ros/basalt_types.hpp"

namespace basalt_ros
{
class VIOPublisher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit VIOPublisher(rclcpp::Node * node);

  void publish(const BasaltPoseVelBiasState::Ptr & data);

private:
  // ----- variables --
  rclcpp::Node * node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadCaster_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pub_;
  std::array<double, 36> cov_;
  nav_msgs::msg::Odometry msg_;
  bool extraTF_{false};
  Eigen::Vector3d T_extra_;     // extra translation:T_world_basaltworld
  Eigen::Quaterniond q_extra_;  // extra quaternion: q_world_basaltworld
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__VIO_PUBLISHER_HPP_
