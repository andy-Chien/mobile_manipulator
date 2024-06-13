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
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit, OnProcessStart

from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    mobile_type = LaunchConfiguration("mobile_type")
    controller_config_package = LaunchConfiguration("controller_config_package")
    world_file = LaunchConfiguration("world_file")
    mobile_prefix = LaunchConfiguration("mobile_prefix")
    initial_controller = LaunchConfiguration("initial_controller")
    pos_x = LaunchConfiguration("pos_x")
    pos_y = LaunchConfiguration("pos_y")
    pos_z = LaunchConfiguration("pos_z")
    pose_xyz = LaunchConfiguration("pose_xyz")
    pose_rpy = LaunchConfiguration("pose_rpy")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    joy_dev = LaunchConfiguration("joy_dev")
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")
    description_file = LaunchConfiguration("description_file")

    if any(x.perform(context) != "0.0" for x in [pos_x, pos_y, pos_z]):
        raise Exception('Param pos_x, pos_y, pos_z are deprecated!')

    use_sim_time = sim_gazebo # use sim time only when using gazebo

    controllers_config = PathJoinSubstitution(
        [FindPackageShare(controller_config_package), "config", "mm_controllers.yaml"]
    )

    cp = ParameterFile(controllers_config, allow_substs=True)
    with open(cp.evaluate(context), 'r') as f, open(
    '/tmp/launch____controllers_config.yaml', 'w') as h:
        h.write(f.read())

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mm_description"), "urdf", description_file]
            ),
            " ",
            "name:=",
            "mobile_base",
            " ",
            "robot_type:=",
            mobile_type,
            " ",
            "prefix:=",
            mobile_prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "simulation_controllers:=",
            '/tmp/launch____controllers_config.yaml',
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    mb_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["mb_servo_controller", "-c", "controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mb_servo_controller" else "true")
    )

    path_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["mobile_path_controller", "-c", "controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mobile_path_controller" else "true")
    )

    active_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=[initial_controller, "-c", "controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("mm_description"), "/launch", "/gzserver.launch.py"]
        ),
        launch_arguments={'world': PathJoinSubstitution(
        [FindPackageShare("mm_description"), "world", world_file])}.items(),
        condition=IfCondition(sim_gazebo)
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
        ),
        condition=IfCondition(sim_gazebo)
    )

    # Spawn robot

    pose_xyz_text = pose_xyz.perform(context).split(' ')
    pose_rpy_text = pose_rpy.perform(context).split(' ')
    if len(pose_xyz_text) != 3: pose_xyz_text = ['0.0', '0.0', '0.0']
    if len(pose_rpy_text) != 3: pose_rpy_text = ['0.0', '0.0', '0.0']
    

    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_mm",
        namespace=namespace,
        arguments=[
            "-entity", "mm",
            "-topic", "robot_description",
            "-x", pose_xyz_text[0],
            "-y", pose_xyz_text[1],
            "-z", pose_xyz_text[2],
            "-R", pose_rpy_text[0],
            "-P", pose_rpy_text[1],
            "-Y", pose_rpy_text[2],
            "-timeout", "500",
        ],
        output="screen",
        condition=IfCondition(sim_gazebo)
    )

    joy_stick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("mm_joystick"), "/launch", "/mm_joystick.launch.py"]
        ),
        launch_arguments={"use_sim_time": use_sim_time, "joy_dev": joy_dev}.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[
            robot_description, 
            ParameterFile(controllers_config, allow_substs=True)
        ],
        output="both",
        condition=UnlessCondition(sim_gazebo)
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mm_description"), "config/rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        namespace=namespace,
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_view_robot",
        output="log",
        arguments=["-d", rviz_config],
    )

    set_base_init_pose = Node(
        package="mm_controllers",
        executable="base_init_pose_setter.py",
        output="both",
        namespace=namespace,
        parameters=[{
            "use_sim_time": use_sim_time,
            "pose_xyz": pose_xyz,
            "pose_rpy": pose_rpy,
        }],
    )

    sim_gazebo_bool = sim_gazebo.perform(context) == "true" or sim_gazebo.perform(context) == "True"

    load_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[
                mb_servo_controller_spawner,
                path_controller_spawner,
                active_controller_spawner,
                joint_state_broadcaster_spawner,
            ],
        )
    ) if sim_gazebo_bool else RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                mb_servo_controller_spawner,
                path_controller_spawner,
                active_controller_spawner,
                joint_state_broadcaster_spawner,
            ],
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        gazebo_spawn_robot,
        joy_stick,
        load_controller,
        set_base_init_pose,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config_package",
            default_value="mm_description",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="two_walls.world",
            description="world description file for gazebo simulation.",
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
            "mobile_prefix",
            default_value="mobile_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_controller",
            default_value="mb_servo_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mobile_type",
            default_value="delta_mb",
            description="mobile base type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev",
            default_value="js0",
            description="mobile base type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pos_x",
            default_value="0.0",
            description="x value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pos_y",
            default_value="0.0",
            description="y value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pos_z",
            default_value="0.0",
            description="z value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_xyz",
            default_value="0.0 0.0 0.0",
            description="xyz value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_rpy",
            default_value="0.0 0.0 0.0",
            description="rpy value of position for init the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Using gazebo or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Using fake hardware or not",
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
            default_value="false", 
            description="Launch RViz?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="mobile_base/load_swerve_drive.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
