# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    joy_dev = LaunchConfiguration("joy_dev")
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    arm_prefix = LaunchConfiguration("arm_prefix")
    description_file = LaunchConfiguration("description_file")
    srdf_file = LaunchConfiguration("srdf_file")

    mobile_manipulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("mm_description"), "/launch", "/mobile_manipulator.launch.py"]
        ),
        launch_arguments={
            "use_fake_hardware": "true",
            "joy_dev": joy_dev,
            "namespace": namespace,
            "launch_rviz": launch_rviz,
            "arm_prefix": arm_prefix,
            "description_file": description_file,
            "srdf_file": srdf_file,
            "initial_controller": "mm_trajectory_controller"}.items(),
    )

    return [mobile_manipulator]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value="js0",
            description="mobile base type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace", 
            default_value="", 
            description="Namespace for node"
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
            "arm_prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="load_mobile_manipulator.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_file",
            default_value="robot.srdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
