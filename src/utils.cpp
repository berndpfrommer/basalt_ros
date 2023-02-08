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

#include "basalt_ros/utils.hpp"

// needed for loading calibration via cereal
#include <basalt/serialization/headers_serialization.h>

#include <fstream>

namespace basalt_ros
{
void load_calib(
  rclcpp::Node * node, basalt::Calibration<double> * calib,
  const std::string & calib_path)
{
  if (calib_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "no calib file specified!");
    throw std::invalid_argument("no calibration file specified");
  }
  std::ifstream os(calib_path, std::ios::binary);
  if (os.is_open()) {
    cereal::JSONInputArchive archive(os);
    archive(*calib);
    RCLCPP_INFO_STREAM(
      node->get_logger(), "loaded calibration file with "
                            << calib->intrinsics.size() << " cameras");
  } else {
    RCLCPP_ERROR_STREAM(
      node->get_logger(), "could not load calibration file " << calib_path);
    throw std::ios_base::failure(
      "could not find calibration file: " + calib_path);
  }
}

void load_calib_and_config(
  rclcpp::Node * node, basalt::Calibration<double> * calib,
  basalt::VioConfig * config)
{
  const std::string calib_path =
    node->declare_parameter("calibration_file", "");
  load_calib(node, calib, calib_path);
  // load config
  config->optical_flow_skip_frames = 1;
  const std::string vioConfigFile =
    node->declare_parameter("vio_config_file", "");
  if (!vioConfigFile.empty()) {
    config->load(vioConfigFile);
  }
  // overwrite debug flags if requested
  if (!node->has_parameter("debug_vio")) {
    config->vio_debug = node->declare_parameter("debug_vio", false);
  } else {
    node->get_parameter("debug_vio", config->vio_debug);
  }
}
}  // namespace basalt_ros
