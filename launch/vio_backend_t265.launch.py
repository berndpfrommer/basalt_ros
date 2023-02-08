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
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create simple node."""
    split_imu = (LaunchConfig('imu_topic').perform(context) == "")
    parameters = [
        {'calibration_file': pkg_file(LaunchConfig('calib_file')),
         'vio_config_file': pkg_file(LaunchConfig('config_file')),
         'has_split_accel_and_gyro_topics': split_imu}]
    remappings = [
        ('optical_flow', LaunchConfig('flow_topic'))]

    if split_imu:
        remappings.append(('gyro', LaunchConfig('gyro_topic')))
        remappings.append(('accel', LaunchConfig('accel_topic')))
    else:
        remappings.append(('imu', LaunchConfig('imu_topic')))
    node = Node(
        package='basalt_ros',
        executable='vio_backend_node',
        output='screen',
        name='vio_backend',
        namespace='basalt',
        parameters=parameters,
        remappings=remappings)
    return [node]


def generate_launch_description():
    """Create simple node by calling opaque function."""
    return launch.LaunchDescription([
        LaunchArg('calib_file',
                  default_value=['t265_calib.json'],
                  description='configuration file'),
        LaunchArg('config_file',
                  default_value=['stereo_cam_vio_config.json'],
                  description='camera/imu calibration file'),
        LaunchArg('flow_topic',
                  default_value=['optical_flow'],
                  description='optical flow topic'),
        LaunchArg('gyro_topic',
                  default_value=['/t265/gyro/sample'],
                  description='gyro topic'),
        LaunchArg('accel_topic',
                  default_value=['/t265/accel/sample'],
                  description='accel topic'),
        LaunchArg('imu_topic',
                  default_value=[''],
                  description='combined accel/gyro topic'),
        OpaqueFunction(function=launch_setup)
        ])
