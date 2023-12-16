# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_controller = LaunchConfiguration('launch_controller', default='true')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file_name')
    navigation_config_package = LaunchConfiguration('navigation_config_package')
    this_pkg_dir = FindPackageShare(navigation_config_package)

    map_dir = PathJoinSubstitution(
        [this_pkg_dir, "map", map_file]
    )

    params_dir = PathJoinSubstitution(
        [this_pkg_dir, "param", params_file]
    )

    rviz_config_dir = PathJoinSubstitution(
        [this_pkg_dir, "rviz", "rviz.rviz"]
    )

    navigation_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [this_pkg_dir, '/launch' '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'launch_controller': launch_controller,
            'params_file': params_dir}.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen',
    )

    nodes_to_start = [navigation_bringup, rviz_node]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='two_walls.yaml',
            description='file name of map file to load',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file_name',
            default_value='yhs_fw01.yaml',
            description='file name of param file to load',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "navigation_config_package",
            default_value="mm_navigation_config",
            description="Navigation config package with robot param and map files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_controller",
            default_value="true",
            description="Launch Nav2 native controller or not",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
