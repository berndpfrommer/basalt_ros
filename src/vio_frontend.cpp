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

#include "basalt_ros/vio_frontend.hpp"

#include <fstream>
#include <ios>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <thread>

#include "basalt_ros/utils.hpp"

namespace basalt_ros
{
VIOFrontEnd::VIOFrontEnd(rclcpp::Node * node) : node_(node)
{
  basalt::Calibration<double> calib;
  basalt::VioConfig config;
  basalt_ros::load_calib_and_config(node_, &calib, &config);
  imageSub_ =
    std::make_shared<ImageSubscriber>(node_, "left_image", "right_image");
  opticalFlow_ = basalt::OpticalFlowFactory::getOpticalFlow(config, calib);
  // connect image sub output queue to optical flow input queue
  imageSub_->setQueue(&(opticalFlow_->input_queue));

  if (node_->declare_parameter<bool>("publish_optical_flow", true)) {
    opticalFlowPub_ =
      std::make_shared<OpticalFlowPublisher>(node_, "optical_flow");
    // set the output queue of the optical flow to be
    // the input queue of the publisher
    opticalFlow_->output_queue = opticalFlowPub_->getInputQueue();
    opticalFlowPub_->start();
  }
}

}  // namespace basalt_ros

RCLCPP_COMPONENTS_REGISTER_NODE(basalt_ros::VIOFrontEndNode)
