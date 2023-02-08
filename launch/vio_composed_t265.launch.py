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
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def pkg_file(fname):
    return PathJoinSubstitution(
        [FindPackageShare('basalt_ros'), 'config', fname])


def launch_setup(context, *args, **kwargs):
    """Create container."""
    split_imu = (LaunchConfig('imu_topic').perform(context) == "")

    common_parameters = \
        {'calibration_file': pkg_file(LaunchConfig('calib_file')),
         'vio_config_file': pkg_file(LaunchConfig('config_file'))}
    frontend_parameters = common_parameters
    backend_parameters = {**common_parameters,
                          **{'has_split_accel_and_gyro_topics': split_imu}}

    frontend_remappings = [
        ('left_image', [LaunchConfig('left_cam'), '/image_raw']),
        ('right_image', [LaunchConfig('right_cam'), '/image_raw'])]
    backend_remappings = [
        ('optical_flow', LaunchConfig('flow_topic'))]

    if split_imu:
        backend_remappings.append(('gyro', LaunchConfig('gyro_topic')))
        backend_remappings.append(('accel', LaunchConfig('accel_topic')))
    else:
        backend_remappings.append(('imu', LaunchConfig('imu_topic')))
    container = ComposableNodeContainer(
            name='basalt_vio_container',
            namespace='basalt',
            package='rclcpp_components',
            executable='component_container',
            # prefix=['xterm -e gdb -ex run --args'],
            composable_node_descriptions=[
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOFrontEndNode',
                    name='vio_frontend',
                    namespace='basalt',
                    parameters=[frontend_parameters],
                    remappings=frontend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='basalt_ros',
                    plugin='basalt_ros::VIOBackEndNode',
                    name='vio_backend',
                    namespace='basalt',
                    parameters=[backend_parameters],
                    remappings=backend_remappings,
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen')
    return [container]


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
