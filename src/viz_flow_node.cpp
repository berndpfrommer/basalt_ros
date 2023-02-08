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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <basalt_vio_msgs/msg/optical_flow_result.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::placeholders;

class VizFlow : public rclcpp::Node
{
public:
  typedef basalt_vio_msgs::msg::OpticalFlowResult OptFlow;
  typedef sensor_msgs::msg::Image Image;
  typedef message_filters::sync_policies::ExactTime<Image, OptFlow> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  explicit VizFlow(const rclcpp::NodeOptions & options)
  : Node("viz_flow", options)

  {
    sync_ = std::make_shared<Sync>(10);
    const auto qos = rmw_qos_profile_default;
    imageSub_ = std::make_shared<message_filters::Subscriber<Image>>(
      this, "left_image", qos);
    flowSub_ = std::make_shared<message_filters::Subscriber<OptFlow>>(
      this, "optical_flow", qos);
    imagePub_ = image_transport::create_publisher(this, "flow_image", qos);
    sync_ = std::make_shared<Sync>(SyncPolicy(10), *imageSub_, *flowSub_);
    sync_->registerCallback(std::bind(&VizFlow::flowCallback, this, _1, _2));
    drawTracks_ = this->declare_parameter<bool>("show_tracks", false);
  }

private:
  void flowCallback(
    const Image::ConstSharedPtr & imgMsg,
    const OptFlow::ConstSharedPtr & optFlow)
  {
    const auto imgPtr =
      cv_bridge::toCvShare(imgMsg, sensor_msgs::image_encodings::MONO8);
    const int h = imgPtr->image.rows;
    const int w = imgPtr->image.cols;
    cv::Mat img(h, w, CV_8UC3);
    cv::cvtColor(imgPtr->image, img, CV_GRAY2RGB);
    const cv::Scalar newKeypointColor(0, 255, 255);
    const cv::Scalar oldKeypointColor(0, 255, 0);
    const cv::Scalar trackColor(255, 255, 0);
    for (size_t obs_idx = 0; obs_idx < optFlow->observations.size();
         obs_idx++) {
      const auto & msg_obs = optFlow->observations[obs_idx];
      for (size_t kp_idx = 0; kp_idx < msg_obs.keypoints.size(); kp_idx++) {
        const auto & kp = msg_obs.keypoints[kp_idx];
        const cv::Point2f p(kp.affine_transform[4], kp.affine_transform[5]);
        auto itb = tracks_.insert({kp.id, std::vector<cv::Point2f>()});
        auto & track = itb.first->second;
        track.push_back(p);
        const cv::Scalar kpc = itb.second ? newKeypointColor : oldKeypointColor;
        if (drawTracks_) {
          for (size_t i = 0; i < track.size() - 1; i++) {
            cv::line(img, track[i], track[i + 1], trackColor, 1, cv::LINE_8);
          }
        }
        cv::circle(img, p, 3, kpc, -1);
      }
    }
    cv_bridge::CvImage outMsg(imgMsg->header, "bgr8", img);
    imagePub_.publish(outMsg.toImageMsg());
  }
  // ----------- variables ---------
  std::shared_ptr<message_filters::Subscriber<Image>> imageSub_;
  std::shared_ptr<message_filters::Subscriber<OptFlow>> flowSub_;
  image_transport::Publisher imagePub_;
  std::map<uint32_t, std::vector<cv::Point2f>> tracks_;
  std::shared_ptr<Sync> sync_;
  bool drawTracks_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VizFlow>(rclcpp::NodeOptions());
  rclcpp::spin(node);  // should not return
  return (0);
}
