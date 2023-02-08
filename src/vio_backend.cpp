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

#include "basalt_ros/vio_backend.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "basalt_ros/utils.hpp"

namespace basalt_ros
{
VIOBackEnd::VIOBackEnd(rclcpp::Node * node) : node_(node)
{
  const auto logger = node_->get_logger();
  basalt::Calibration<double> calib;
  basalt::VioConfig config;
  basalt_ros::load_calib_and_config(node_, &calib, &config);

  RCLCPP_INFO_STREAM(
    logger, "VIO debug: " << (config.vio_debug ? " TRUE" : " FALSE"));

  // create VIO object
  vio_ = basalt::VioEstimatorFactory::getVioEstimator(
    config, calib, basalt::constants::g, true, true /* use double */);
  vio_->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  flowSub_ = std::make_shared<OpticalFlowSubscriber>(node_, "optical_flow");
  flowSub_->setQueue(&vio_->vision_data_queue);

  // subscribe to IMU and pipe into the vio
  const bool splitAccelGyro =
    node_->declare_parameter("has_split_accel_and_gyro_topics", false);

  std::vector<std::string> imu_topics;
  if (splitAccelGyro) {
    imu_topics = {"gyro", "accel"};
  } else {
    imu_topics = {"imu"};
  }
  imuSub_ = std::make_shared<IMUSubscriber>(node_, imu_topics);
  imuSub_->setQueue(&(vio_->imu_data_queue));

  // create publisher for odom
  publisher_ = std::make_shared<VIOPublisher>(node_);

  // connect the VIO output to our own queue for publishing
  vio_->out_state_queue = &outputQueue_;
  thread_ = std::thread(&VIOBackEnd::publishingThread, this);
  RCLCPP_INFO(node_->get_logger(), "backend started!");
}

void VIOBackEnd::publishingThread()
{
  BasaltPoseVelBiasState::Ptr data;
  for (;;) {
    outputQueue_.pop(data);
    if (!data.get()) {
      RCLCPP_INFO(node_->get_logger(), "exiting!");
      break;
    }
    publisher_->publish(data);
  }
}

}  // namespace basalt_ros

RCLCPP_COMPONENTS_REGISTER_NODE(basalt_ros::VIOBackEndNode)
