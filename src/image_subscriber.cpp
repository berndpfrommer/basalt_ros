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

#include "basalt_ros/image_subscriber.hpp"

#include <functional>
#include <stdexcept>

using namespace std::placeholders;

namespace basalt_ros
{
ImageSubscriber::ImageSubscriber(
  rclcpp::Node * node, const std::string & topic_left,
  const std::string & topic_right)
: node_(node)
{
  subs_.push_back(
    std::make_shared<message_filters::Subscriber<Image>>(node_, topic_left));
  subs_.push_back(
    std::make_shared<message_filters::Subscriber<Image>>(node, topic_right));
  sync_ = std::make_shared<message_filters::TimeSynchronizer<Image, Image>>(
    *subs_[0], *subs_[1], 100);
  sync_->registerCallback(std::bind(&ImageSubscriber::callback, this, _1, _2));
  lastTime_ = node_->now();
}

void ImageSubscriber::callback(
  const ImageConstPtr & left, const ImageConstPtr & right)
{
  basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput);

  const int NUM_CAMS = 2;
  data->img_data.resize(NUM_CAMS);
  data->t_ns =
    left->header.stamp.sec * 1000000000LL + left->header.stamp.nanosec;
  const ImageConstPtr msg[2] = {left, right};
  for (int i = 0; i < NUM_CAMS; i++) {
    const auto & img = *msg[i];
    data->img_data[i].exposure =
      0;  /// XXX  RS2_FRAME_METADATA_ACTUAL_EXPOSURE * 1e-6

    data->img_data[i].img.reset(
      new basalt::ManagedImage<uint16_t>(img.width, img.height));

    const uint8_t * data_in = (const uint8_t *)&img.data[0];
    uint16_t * data_out = data->img_data[i].img->ptr;
    // TODO(Bernd): this 8-bit to 16bit conversion can probably be done
    // more efficiently with opencv
    size_t full_size = img.width * img.height;
    for (size_t j = 0; j < full_size; j++) {
      int val = data_in[j];
      val = val << 8;
      data_out[j] = val;
    }
  }

  framesReceived_++;
  if (framesReceived_ == 100) {
    const auto t = node_->now();
    const auto dt = t - lastTime_;
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "received imag frame rate: " << framesReceived_ * 1e9 / dt.nanoseconds());
    framesReceived_ = 0;
    lastTime_ = t;
  }

  if (queue_) {
    if (!queue_->try_push(data)) {
      RCLCPP_WARN_STREAM(
        node_->get_logger(),
        "image frame " << data->t_ns << " dropped due to queue overflow");
    }
    max_q_ = std::max(queue_->size(), max_q_);
  }
}
}  // namespace basalt_ros
