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

#include "basalt_ros/optical_flow_subscriber.hpp"

#include <functional>

using namespace std::placeholders;

namespace basalt_ros
{
OpticalFlowSubscriber::OpticalFlowSubscriber(
  rclcpp::Node * node, const std::string & topic)
: node_(node)
{
  sub_ = node_->create_subscription<OpticalFlowMsg>(
    topic, 10, std::bind(&OpticalFlowSubscriber::callback, this, _1));
}

void OpticalFlowSubscriber::callback(const OpticalFlowMsgConstPtr msg)
{
  if (!queue_) {
    return;
  }
  basalt::OpticalFlowResult::Ptr data(new basalt::OpticalFlowResult());

  data->t_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
  data->observations.resize(msg->observations.size());
  for (size_t obs_idx = 0; obs_idx < msg->observations.size(); obs_idx++) {
    const auto & msg_obs = msg->observations[obs_idx];
    auto & data_obs = data->observations[obs_idx];
    for (size_t kp_idx = 0; kp_idx < msg_obs.keypoints.size(); kp_idx++) {
      const auto kp_id = msg_obs.keypoints[kp_idx].id;
      // copy affine transform
      for (int i = 0; i < 6; i++) {
        data_obs[kp_id].data()[i] =
          msg_obs.keypoints[kp_idx].affine_transform[i];
      }
    }
  }
  if (queue_) {
    if (!queue_->try_push(data)) {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "optical flow data dropped due to overflow: q_len = "
          << queue_->size());
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros
