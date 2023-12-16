# Copyright (c) 2021 PickNik, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    mobile_type = LaunchConfiguration("mobile_type")
    can_port = LaunchConfiguration("can_port")
    hw_read_freq = LaunchConfiguration("hw_read_freq")
    hw_write_freq = LaunchConfiguration("hw_write_freq")
    hw_wheel_diam = LaunchConfiguration("hw_wheel_diam")
    # General arguments
    controller_config_package = LaunchConfiguration("controller_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    initial_mobile_base_controller = LaunchConfiguration("initial_mobile_base_controller")
    activate_mobile_base_controller = LaunchConfiguration("activate_mobile_base_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")

    controllers_config = PathJoinSubstitution(
        [FindPackageShare(controller_config_package), "config", "fw01_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            "fw01",
            " ",
            "mobile_type:=",
            mobile_type,
            " ",
            "can_port:=",
            can_port,
            " ",
            "hw_read_freq:=",
            hw_read_freq,
            " ",
            "hw_write_freq:=",
            hw_write_freq,
            " ",
            "hw_wheel_diam:=",
            hw_wheel_diam,
            " ",
            "mobile_prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "simulation_controllers:=",
            controllers_config,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    # robot_description2 = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # print("Test")

    # print(robot_description)
    # print(robot_description2)   
    # Command(robot_description_content)
    # print("Test3")

    controller_config_file = PathJoinSubstitution(
        [FindPackageShare(controller_config_package), "config", controllers_file]
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    # define update rate
    # update_rate_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare(controller_config_package),
    #         "config",
    #         ur_type.perform(context) + "_update_rate.yaml",
    #     ]
    # )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_file],
        output="screen",
        #condition=IfCondition(use_fake_hardware),
    )
    


    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, controller_config_file],
    #     output="both",
    # )

    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )

    # ur_control_node = Node(
    #     package="ur_robot_driver",
    #     executable="ur_ros2_control_node",joint_state_broadcaster
    #     parameters=[robot_description, controller_config_file],
    #     output="screen",
    #     condition=UnlessCondition(use_fake_hardware),
    # )

    # UR overrides controller_manager and ControllerStopper/Start/Switch service. We do not need that
    # controller_stopper_node = Node(
    #     package="ur_robot_driver",
    #     executable="controller_stopper_node",
    #     name="controller_stopper",
    #     output="screen",
    #     emulate_tty=True,
    #     condition=UnlessCondition(use_fake_hardware),
    #     parameters=[
    #         {"headless_mode": headless_mode},
    #         {"joint_controller_active": activate_joint_controller},
    #         {
    #             "consistent_controllers": [
    #                 "io_and_status_controller",
    #                 "force_torque_sensor_broadcaster",
    #                 "joint_state_broadcaster",
    #                 "speed_scaling_state_broadcastfrom launch.actions import RegisterEventHandlerer",
    #             ]
    #         },
    #     ],
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mobile_io_and_status_controller", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mobile_standard_driver_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
         )
    )     

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
         )
    )       

    # To Spawn:
    # 1. Controller Manager
    # 2. Robot State publisher, Joint_State_Broadcaster
    # 3. IO_and_Status_Controller
    # 4. 
    # 5. Robot_Controller : Steer_Controller (active) and freedrive_controller (inactive)
    # Play around with delay


    # There may be other controllers of the joints, but this is the initially-started one
    initial_mobile_base_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_mobile_base_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_mobile_base_controller),
    )
    initial_mobile_base_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_mobile_base_controller, "-c", "/controller_manager", "--inactive"],
        condition=UnlessCondition(activate_mobile_base_controller),
    )

    nodes_to_start = [
        control_node, #okay
        #ur_control_node, controller_stopper_node,
        robot_state_publisher_node, #okay
        rviz_node, #okay
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        initial_mobile_base_controller_spawner_stopped, #okay
        initial_mobile_base_controller_spawner_started, #okay
    ]

    start = Node(
        package="urdf_tutorial",
        executable="display.launch.py",
        arguments=["model:=fw01.urdf.xacro", "name:=fw01"],
    )

    nodes_to_start2 = [
        start,
    ]

    return nodes_to_start 


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "mobile_type",
            description="Type/series of used mobile robot. Option only yhs_fw01",
            choices=["yhs_fw01"],
            default_value="yhs_fw01",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="CAN port for mobile robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hw_read_freq",
            default_value="100",
            description="Default Read Freq is 100Hz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hw_write_freq",
            default_value="100",
            description="Default Write Freq is 100Hz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hw_wheel_diam",
            default_value="0.24",
            description="Wheel Diameter is 24cm",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config_package",
            default_value="fw01_controllers",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="fw01_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="fw01_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="fw01.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_mobile_base_controller",
            default_value="steer_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_mobile_base_controller",
            default_value="false", #change to true later
            description="Activate initial mobile base controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
