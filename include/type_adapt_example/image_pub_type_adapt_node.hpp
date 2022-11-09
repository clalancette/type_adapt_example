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

#ifndef TYPE_ADAPT_EXAMPLE__IMAGE_PUB_TYPE_ADAPT_NODE_HPP_
#define TYPE_ADAPT_EXAMPLE__IMAGE_PUB_TYPE_ADAPT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "type_adapt_example/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "type_adapt_example/visibility_control.h"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  type_adapt_example::ROSCvMatContainer,
  sensor_msgs::msg::Image);

namespace type_adapt_example
{

class ImagePubTypeAdaptNode : public rclcpp::Node
{
public:
  TYPE_ADAPT_EXAMPLE_PUBLIC
  explicit ImagePubTypeAdaptNode(const rclcpp::NodeOptions & options);

  virtual ~ImagePubTypeAdaptNode();

private:
  rclcpp::Publisher<type_adapt_example::ROSCvMatContainer>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace type_adapt_example

#endif  // TYPE_ADAPT_EXAMPLE__IMAGE_PUB_TYPE_ADAPT_NODE_HPP_
