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

#include "basalt_ros/imu_publisher.hpp"

namespace basalt_ros
{
IMUPublisher::IMUPublisher(rclcpp::Node * node, const std::string & topic)
: node_(node), topic_(topic)
{
  pub_ = node_->create_publisher<ImuMsg>(topic, 10);
  lastTime_ = node_->now();
}

void IMUPublisher::processingThread()
{
  BasaltImuData::Ptr data;
  for (;;) {
    inputQueue_.pop(data);
    if (!data) {
      break;  // done!
    }
    if (node_->count_subscribers(topic_) != 0) {
      ImuMsg msg;
      msg.header.frame_id = "frame_id";  // XXX make configurable
      msg.header.stamp.sec = data->t_ns / 1000000000LL;
      msg.header.stamp.nanosec = data->t_ns % 1000000000LL;

      msg.linear_acceleration.x = data->accel(0);
      msg.linear_acceleration.y = data->accel(1);
      msg.linear_acceleration.z = data->accel(2);

      msg.angular_velocity.x = data->gyro(0);
      msg.angular_velocity.y = data->gyro(1);
      msg.angular_velocity.z = data->gyro(2);
      pub_->publish(msg);
    }
    framesPublished_++;
    if (framesPublished_ == 1000) {
      const auto t = node_->now();
      const auto dt = t - lastTime_;
      RCLCPP_INFO_STREAM(
        node_->get_logger(), "published imu frame rate: "
                               << framesPublished_ * 1e9 / dt.nanoseconds());
      framesPublished_ = 0;
      lastTime_ = t;
    }
  }
}

void IMUPublisher::start()
{
  thread_ = std::thread(&IMUPublisher::processingThread, this);
}

}  // namespace basalt_ros
