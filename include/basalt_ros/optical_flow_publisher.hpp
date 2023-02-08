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

#ifndef BASALT_ROS__OPTICAL_FLOW_PUBLISHER_HPP_
#define BASALT_ROS__OPTICAL_FLOW_PUBLISHER_HPP_

#include <basalt/optical_flow/optical_flow.h>
#include <tbb/concurrent_queue.h>

#include <basalt_vio_msgs/msg/optical_flow_result.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace basalt_ros
{
class OpticalFlowPublisher
{
public:
  OpticalFlowPublisher(rclcpp::Node * node, const std::string & topic);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> *
  getInputQueue()
  {
    return (&inputQueue_);
  }
  void start();

private:
  void processingThread();
  // ----- variables --
  rclcpp::Node * node_;
  std::string topic_;
  std::shared_ptr<rclcpp::Publisher<basalt_vio_msgs::msg::OpticalFlowResult>>
    pub_;
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> inputQueue_;
  std::thread thread_;
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__OPTICAL_FLOW_PUBLISHER_HPP_
