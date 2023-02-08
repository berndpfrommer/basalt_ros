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

#ifndef BASALT_ROS__IMU_PUBLISHER_HPP_
#define BASALT_ROS__IMU_PUBLISHER_HPP_

#include <tbb/concurrent_queue.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

#include "basalt_ros/basalt_types.hpp"

namespace basalt_ros
{
class IMUPublisher
{
public:
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef std::shared_ptr<const ImuMsg> ImuMsgConstPtr;
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<BasaltImuData>>
    ImuDataQueue;
  explicit IMUPublisher(rclcpp::Node * node, const std::string & topic);

  ImuDataQueue * getInputQueue() { return (&inputQueue_); }
  void start();

private:
  void processingThread();
  // ----- variables --
  rclcpp::Node * node_;
  std::string topic_;
  std::shared_ptr<rclcpp::Publisher<ImuMsg>> pub_;
  ImuDataQueue inputQueue_;
  std::thread thread_;
  uint64_t framesPublished_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__IMU_PUBLISHER_HPP_
