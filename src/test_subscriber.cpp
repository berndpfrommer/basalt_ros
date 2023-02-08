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

#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "basalt_ros/imu_subscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
    rclcpp::Node::make_shared("test_subscriber");
  const std::vector<std::string> topics = {"gyro", "accel"};
  basalt_ros::IMUSubscriber imuSub(node.get(), topics);
  RCLCPP_INFO(node->get_logger(), "test subscriber started!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
