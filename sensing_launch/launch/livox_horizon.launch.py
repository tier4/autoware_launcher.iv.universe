
# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import launch
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable

def launch_setup(context, *args, **kwargs):
    bd_code_param_path = LaunchConfiguration('bd_code_param_path').perform(context)
    with open(bd_code_param_path, 'r') as f:
        bd_code_param = yaml.safe_load(f)['/**']['ros__parameters']

    # livox driver
    livox_driver_component = ComposableNode(
        package='livox_ros2_driver',
        plugin='livox_ros::LivoxDriver',
        name='livox_driver',
        parameters=[
            {
                'xfe_format': LaunchConfiguration('xfe_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': LaunchConfiguration('data_src'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': LaunchConfiguration('output_type'),
                'lvx_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path': LaunchConfiguration('user_config_path'),
                'frame_id': LaunchConfiguration('sensor_frame'),
                'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME',
                                                    default_value='False'),
            },
            bd_code_param,
        ]
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            livox_driver_component
        ],
        output='screen',
        parameters=[{
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }],
    )

    return [container]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('xfe_format', '0')
    add_launch_arg('multi_topic', '0')
    add_launch_arg('data_src', '0')
    add_launch_arg('publish_freq', '10.0')
    add_launch_arg('output_type', '0')
    add_launch_arg('lvx_file_path', 'livox_test.lvx')
    add_launch_arg('user_config_path', os.path.join(get_package_share_directory(
        "livox_ros2_driver"), "config/livox_lidar_config.json"))
    add_launch_arg('bd_code_param_path')
    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('sensor_frame', 'livox_frame')

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
