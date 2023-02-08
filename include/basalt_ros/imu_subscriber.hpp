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

#ifndef BASALT_ROS__IMU_SUBSCRIBER_HPP_
#define BASALT_ROS__IMU_SUBSCRIBER_HPP_

#include <tbb/concurrent_queue.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

#include "basalt_ros/basalt_types.hpp"

namespace basalt_ros
{
class IMUSubscriber
{
public:
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef std::shared_ptr<const ImuMsg> ImuMsgConstPtr;
  typedef tbb::concurrent_bounded_queue<std::shared_ptr<BasaltImuData>>
    ImuDataQueue;
  typedef std::shared_ptr<ImuDataQueue> ImuDataQueuePtr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUSubscriber(rclcpp::Node * node, const std::vector<std::string> & topics);

  ~IMUSubscriber() {}

  ImuDataQueue * getQueue() { return queue_; }
  void setQueue(ImuDataQueue * q) { queue_ = q; }
  void printFrameRate();

private:
  void callback_gyro(const ImuMsgConstPtr msg);
  void callback_accel(const ImuMsgConstPtr msg);
  void callback_combined(const ImuMsgConstPtr msg);
  // ----- variables --
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<ImuMsg>::SharedPtr gyroSub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr accelSub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr combinedSub_;
  std::list<ImuMsgConstPtr> gyroQueue_;
  ImuMsgConstPtr prevAccel_;
  ImuDataQueue * queue_{nullptr};
  int64_t max_q_{0};
  uint64_t combinedFramesReceived_{0};
  rclcpp::Time lastTime_;
};
}  // namespace basalt_ros
#endif  // BASALT_ROS__IMU_SUBSCRIBER_HPP_
