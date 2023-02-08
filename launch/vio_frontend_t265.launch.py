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
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    parameters = [
        {'calibration_file': pkg_file(LaunchConfig('calib_file')),
         'vio_config_file': pkg_file(LaunchConfig('config_file'))}]
    remappings = [
        ('left_image', [LaunchConfig('left_cam'), '/image_raw']),
        ('right_image', [LaunchConfig('right_cam'), '/image_raw'])]

    node = Node(
        package='basalt_ros',
        executable='vio_frontend_node',
        output='screen',
        name='vio_frontend',
        namespace='basalt',
        parameters=parameters,
        remappings=remappings)
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('left_cam',
                  default_value=['/t265/fisheye1'],
                  description='name of left camera'),
        LaunchArg('right_cam',
                  default_value=['/t265/fisheye2'],
                  description='name of right camera'),
        LaunchArg('calib_file', default_value=['t265_calib.json'],
                  description='camera calibration file'),
        LaunchArg('config_file', default_value=['t265_vio_config.json'],
                  description='vio config file'),
        OpaqueFunction(function=launch_setup)
        ])
