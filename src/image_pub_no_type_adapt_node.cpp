// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "type_adapt_example/image_pub_no_type_adapt_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace type_adapt_example
{

ImagePubNoTypeAdaptNode::ImagePubNoTypeAdaptNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_pub_no_type_adapt", options)
{
  auto publish_message =
    [this]() -> void
    {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      // Some example resolutions:
      // D435i RGB - 1920x1080 @ 30 frames/sec
      // D435i Depth - 1280x720 @ 90 frames/sec
      msg->width = 1920;  // width of D435i RGB
      msg->height = 1080;  // height of D435i
      msg->step = msg->width * 3;  // assuming 8 bpp
      msg->encoding = "rgb8";
      size_t size = msg->step * msg->height;
      msg->data.resize(size);
      // TODO(clalancette): Probably fill in some data
      msg->header.frame_id = "image";
      msg->header.stamp = this->now();
      RCLCPP_INFO(this->get_logger(), "Publishing");
      pub_->publish(std::move(msg));
    };
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(33), publish_message);
}

ImagePubNoTypeAdaptNode::~ImagePubNoTypeAdaptNode()
{
}

}  // namespace type_adapt_example

RCLCPP_COMPONENTS_REGISTER_NODE(type_adapt_example::ImagePubNoTypeAdaptNode)
