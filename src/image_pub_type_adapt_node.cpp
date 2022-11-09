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

#include "type_adapt_example/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "type_adapt_example/image_pub_type_adapt_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace type_adapt_example
{

ImagePubTypeAdaptNode::ImagePubTypeAdaptNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("image_pub_type_adapt", options)
{
  auto publish_message =
    [this]() -> void
    {
      cv::Mat frame(1920, 1080, CV_8UC3);
      // TODO(clalancette): Fill this out

      std_msgs::msg::Header header;
      header.frame_id = "image";
      header.stamp = this->now();

      auto container = std::make_unique<type_adapt_example::ROSCvMatContainer>(frame, header);
      RCLCPP_INFO(this->get_logger(), "Publishing");
      pub_->publish(std::move(container));
    };

  pub_ = this->create_publisher<type_adapt_example::ROSCvMatContainer>("image", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(33), publish_message);
}

ImagePubTypeAdaptNode::~ImagePubTypeAdaptNode()
{
}

}  // namespace type_adapt_example

RCLCPP_COMPONENTS_REGISTER_NODE(type_adapt_example::ImagePubTypeAdaptNode)
