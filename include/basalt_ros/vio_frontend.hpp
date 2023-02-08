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

#ifndef BASALT_ROS__VIO_FRONTEND_HPP_
#define BASALT_ROS__VIO_FRONTEND_HPP_

#include <basalt/optical_flow/optical_flow.h>
#include <tbb/concurrent_queue.h>

#include <basalt/calibration/calibration.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "basalt_ros/image_subscriber.hpp"
#include "basalt_ros/optical_flow_publisher.hpp"

namespace basalt_ros
{
class VIOFrontEnd
{
public:
  explicit VIOFrontEnd(rclcpp::Node * node);
  void setOpticalFlowOutputQueue(
    tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> * q)
  {
    opticalFlow_->output_queue = q;
  }

private:
  // ----- variables --
  rclcpp::Node * node_{nullptr};
  basalt::OpticalFlowBase::Ptr opticalFlow_;
  std::shared_ptr<ImageSubscriber> imageSub_;
  std::shared_ptr<OpticalFlowPublisher> opticalFlowPub_;
};

class VIOFrontEndNode : public rclcpp::Node
{
public:
  explicit VIOFrontEndNode(const rclcpp::NodeOptions & options)
  : Node("vio_backend", options)
  {
    frontend_ = std::make_shared<VIOFrontEnd>(this);
  }

private:
  std::shared_ptr<VIOFrontEnd> frontend_;
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__VIO_FRONTEND_HPP_
