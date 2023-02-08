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

#include "basalt_ros/optical_flow_publisher.hpp"

namespace basalt_ros
{
OpticalFlowPublisher::OpticalFlowPublisher(
  rclcpp::Node * node, const std::string & topic)
: node_(node), topic_(topic)
{
  pub_ =
    node_->create_publisher<basalt_vio_msgs::msg::OpticalFlowResult>(topic, 10);
}

void OpticalFlowPublisher::processingThread()
{
  basalt::OpticalFlowResult::Ptr data;
  for (;;) {
    inputQueue_.pop(data);
    if (!data) {
      break;  // done!
    }
    if (node_->count_subscribers(topic_) != 0) {
      basalt_vio_msgs::msg::OpticalFlowResult msg;
      msg.header.frame_id = "frame_id";  // XXX make configurable
      msg.header.stamp.sec = data->t_ns / 1000000000LL;
      msg.header.stamp.nanosec = data->t_ns % 1000000000LL;
      msg.observations.resize(data->observations.size());
      int obs_idx = 0;
      for (const Eigen::aligned_map<
             basalt::KeypointId, Eigen::AffineCompact2f> & map :
           data->observations) {
        msg.observations[obs_idx].keypoints.resize(map.size());
        int kp_idx = 0;
        for (const auto & kp : map) {
          basalt_vio_msgs::msg::KeyPointToAffine kp2Aff;
          kp2Aff.id = kp.first;
          for (int i = 0; i < 6; i++) {
            kp2Aff.affine_transform[i] = kp.second.data()[i];
          }
          msg.observations[obs_idx].keypoints[kp_idx] = kp2Aff;
          kp_idx++;
        }
        obs_idx++;
      }
      pub_->publish(msg);
    }
  }
}

void OpticalFlowPublisher::start()
{
  thread_ = std::thread(&OpticalFlowPublisher::processingThread, this);
}

}  // namespace basalt_ros
