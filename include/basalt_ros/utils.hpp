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

#ifndef BASALT_ROS__UTILS_HPP_
#define BASALT_ROS__UTILS_HPP_

#include <basalt/utils/vio_config.h>

#include <basalt/calibration/calibration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace basalt_ros
{
void load_calib(
  basalt::Calibration<double> * calib, const std::string & calib_path);

void load_calib_and_config(
  rclcpp::Node * node, basalt::Calibration<double> * calib,
  basalt::VioConfig * config);

}  // namespace basalt_ros
#endif  // BASALT_ROS__UTILS_HPP_
