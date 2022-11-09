# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the image pub and sub with type adaptation with composition and intraprocess."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='image_type_adapt_intra_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='type_adapt_example',
                plugin='type_adapt_example::ImageSubTypeAdaptIntraNode',
                name='image_sub_type_adapt_intra',
            ),
            ComposableNode(
                package='type_adapt_example',
                plugin='type_adapt_example::ImagePubTypeAdaptIntraNode',
                name='image_pub_type_adapt_intra',
            ),
        ],
        output='both',
    )

    return LaunchDescription([container])
