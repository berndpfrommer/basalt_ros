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

#ifndef BASALT_ROS__OPTICAL_FLOW_SUBSCRIBER_HPP_
#define BASALT_ROS__OPTICAL_FLOW_SUBSCRIBER_HPP_

#include <basalt/optical_flow/optical_flow.h>
#include <tbb/concurrent_queue.h>

#include <basalt_vio_msgs/msg/optical_flow_result.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace basalt_ros
{
class OpticalFlowSubscriber
{
public:
  typedef basalt_vio_msgs::msg::OpticalFlowResult OpticalFlowMsg;
  typedef std::shared_ptr<const OpticalFlowMsg> OpticalFlowMsgConstPtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OpticalFlowSubscriber(rclcpp::Node * node, const std::string & topic);

  void setQueue(
    tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> * q)
  {
    queue_ = q;
  }
  ~OpticalFlowSubscriber() {}

private:
  void callback(const OpticalFlowMsgConstPtr msg);
  // ----- variables --
  rclcpp::Node * node_;
  rclcpp::Subscription<OpticalFlowMsg>::SharedPtr sub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> * queue_{
    nullptr};

  int64_t max_q_{0};
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__OPTICAL_FLOW_SUBSCRIBER_HPP_
