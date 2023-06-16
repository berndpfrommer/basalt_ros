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

#ifndef BASALT_ROS__VIO_BACKEND_HPP_
#define BASALT_ROS__VIO_BACKEND_HPP_

#include <basalt/optical_flow/optical_flow.h>
#include <basalt/vi_estimator/vio_estimator.h>
#include <tbb/concurrent_queue.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "basalt_ros/basalt_types.hpp"
#include "basalt_ros/imu_subscriber.hpp"
#include "basalt_ros/optical_flow_subscriber.hpp"
#include "basalt_ros/vio_publisher.hpp"

namespace basalt_ros
{
class VIOBackEnd
{
public:
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>
    OpticalFlowInputQueue;
  typedef tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr>
    OpticalFlowResultQueue;
  typedef std::shared_ptr<OpticalFlowInputQueue> OpticalFlowInputQueuePtr;
  explicit VIOBackEnd(rclcpp::Node * node);

  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> *
  getOpticalFlowQueue()
  {
    return (&vio_->vision_data_queue);
  }
  void setOpticalFlowQueue(OpticalFlowResultQueue ** opt_flow_queue);

private:
  void publishingThread();
  // ----- variables --
  rclcpp::Node * node_{nullptr};
  std::shared_ptr<IMUSubscriber> imuSub_;
  std::shared_ptr<OpticalFlowSubscriber> flowSub_;
  basalt::VioEstimatorBase::Ptr vio_;
  std::shared_ptr<VIOPublisher> publisher_;
  tbb::concurrent_bounded_queue<BasaltPoseVelBiasState::Ptr> outputQueue_;
  std::thread thread_;
};

class VIOBackEndNode : public rclcpp::Node
{
public:
  explicit VIOBackEndNode(const rclcpp::NodeOptions & options)
  : Node(
      "vio_backend", rclcpp::NodeOptions(options)
                       .automatically_declare_parameters_from_overrides(true))
  {
    backend_ = std::make_shared<VIOBackEnd>(this);
  }

private:
  std::shared_ptr<VIOBackEnd> backend_;
};

}  // namespace basalt_ros
#endif  // BASALT_ROS__VIO_BACKEND_HPP_
