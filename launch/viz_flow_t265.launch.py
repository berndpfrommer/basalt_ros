# -----------------------------------------------------------------------------
# Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    node = Node(
        package='basalt_ros',
        executable='viz_flow_node',
        output='screen',
        name='viz_flow',
        namespace='basalt',
        parameters=[{'show_tracks': LaunchConfig('show_tracks')}],
        remappings=[
            ('left_image', [LaunchConfig('left_cam'), '/image_raw']),
            ('optical_flow', LaunchConfig('flow_topic'))])
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('left_cam',
                  default_value=['/t265/fisheye1'],
                  description='name of left camera'),
        LaunchArg('show_tracks',
                  default_value=['True'],
                  description='toggle showing of tracks'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        OpaqueFunction(function=launch_setup)
        ])
