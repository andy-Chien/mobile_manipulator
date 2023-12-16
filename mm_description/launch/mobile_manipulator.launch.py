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


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    arm_type = LaunchConfiguration("arm_type")
    mobile_type = LaunchConfiguration("mobile_type")
    safety_limits = LaunchConfiguration("safety_limits")
    controller_config_package = LaunchConfiguration("controller_config_package")
    world_file = LaunchConfiguration("world_file")
    mobile_prefix = LaunchConfiguration("mobile_prefix")
    arm_prefix = LaunchConfiguration("arm_prefix")
    initial_controller = LaunchConfiguration("initial_controller")
    pos_x = LaunchConfiguration("pos_x")
    pos_y = LaunchConfiguration("pos_y")
    pos_z = LaunchConfiguration("pos_z")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    joy_dev = LaunchConfiguration("joy_dev")
    namespace = LaunchConfiguration("namespace")
    launch_rviz = LaunchConfiguration("launch_rviz")


    use_sim_time = sim_gazebo # use sim time only when using gazebo

    controllers_config = PathJoinSubstitution(
        [FindPackageShare(controller_config_package), "config", "mm_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mm_description"), "urdf", "load_mobile_manipulator.xacro"]
            ),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=",
            "mobile_manipulator",
            " ",
            "mobile_type:=",
            mobile_type,
            " ",
            "arm_type:=",
            arm_type,
            " ",
            "mobile_prefix:=",
            mobile_prefix,
            " ",
            "arm_prefix:=",
            arm_prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "simulation_controllers:=",
            controllers_config,
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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    mm_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["mm_servo_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mm_servo_controller" else "true")
    )

    path_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["mobile_path_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mobile_path_controller" else "true")
    )

    mm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["mm_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mm_trajectory_controller" else "true")
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "false" if initial_controller.perform(context) == "mobile_path_controller" else "true")
    )

    active_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=[initial_controller, "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_trajectory_controller_spawner_active = Node(
        package="controller_manager",
        executable="spawner",
        output="both",
        namespace=namespace,
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(
            "true" if initial_controller.perform(context) == "mobile_path_controller" else "false")
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
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_mm",
        arguments=[
            "-entity", "mm",
            "-topic", "robot_description",
            "-x", pos_x,
            "-y", pos_y,
            "-z", pos_z,
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

    mm_ik_service = Node(
        package="mm_ik",
        executable="ik_service_sy",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_config],
        output="both",
        condition=UnlessCondition(sim_gazebo)
    )

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mm_moveit_config"), 
                    "srdf", "robot.srdf.xacro"]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "mobile_manipulator",
            " ",
            "arm_prefix:=",
            arm_prefix,
            " ",
            "mobile_prefix:=",
            mobile_prefix,
            " ",
        ]
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

    # pub_map_odom_init_tf = Node(package = "tf2_ros", 
    #     executable = "static_transform_publisher",
    #     arguments = [
    #         "--frame-id", "map",
    #         "--child-frame-id", "odom",
    #     ]
    # )

    set_base_init_pose = Node(
        package="mm_controllers",
        executable="base_init_pose_setter.py",
        output="both",
        namespace=namespace,
        parameters=[{"use_sim_time": use_sim_time}],
    )

    set_description_param = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/mm_servo_controller', 'robot_description',
             robot_description_content.perform(context)],
        output='screen',
    )
    set_semantic_param = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set', '/mm_servo_controller', 'robot_description_semantic', 
            robot_description_semantic_content.perform(context)
        ],
        output='screen',
    )
    if initial_controller.perform(context) == "mm_servo_controller":
        target_action = active_controller_spawner 
    else:
        target_action = mm_servo_controller_spawner

    set_rdf_param_and_launch_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=target_action,
            on_exit=[
                set_description_param,
                set_semantic_param,
                rviz_node
            ]
        )
    )

    sim_gazebo_bool = sim_gazebo.perform(context) == "true" or sim_gazebo.perform(context) == "True"

    loat_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[
                mm_servo_controller_spawner,
                path_controller_spawner,
                mm_trajectory_controller_spawner,
                joint_trajectory_controller_spawner,
                active_controller_spawner,
                joint_trajectory_controller_spawner_active,
                joint_state_broadcaster_spawner,
            ],
        )
    ) if sim_gazebo_bool else RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                mm_servo_controller_spawner,
                path_controller_spawner,
                mm_trajectory_controller_spawner,
                joint_trajectory_controller_spawner,
                active_controller_spawner,
                joint_trajectory_controller_spawner_active,
                joint_state_broadcaster_spawner,
            ],
        )
    )

    nodes_to_start = [
        control_node,
        # set_rdf_param,
        robot_state_publisher_node,
        mm_ik_service,
        gzserver,
        gzclient,
        gazebo_spawn_robot,
        joy_stick,
        loat_controller,
        # pub_map_odom_init_tf,
        set_base_init_pose,
        set_rdf_param_and_launch_rviz,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
            default_value="ur10e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
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
            "mobile_prefix",
            default_value="mobile_",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_prefix",
            default_value='""',
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
            default_value="mm_servo_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mobile_type",
            default_value="yhs_fw01",
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
            "sim_gazebo",
            default_value="false",
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
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
