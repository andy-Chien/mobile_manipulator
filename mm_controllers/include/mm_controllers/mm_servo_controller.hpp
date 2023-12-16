// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MM_CONTROLLERS__MM_SERVO_CONTROLLER_HPP_
#define MM_CONTROLLERS__MM_SERVO_CONTROLLER_HPP_

#include <string>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/position_ik_request.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include "mm_msgs/msg/servo_control.hpp"
#include "mm_msgs/srv/ik.hpp"
#include "mm_controllers/visibility_control.h"
#include "mm_controllers/mm_data_define.hpp"
#include "mm_controllers/mm_controller_utils.hpp"
#include "mm_controllers/four_wheel_steering_kinematics.hpp"
#include "mm_controller_parameters.hpp"


namespace mm_controllers
{
namespace mcu = mm_controllers::utils;
namespace mcdd = mm_controllers::data_define;
using CmdType = mm_msgs::msg::ServoControl;

/**
 * \brief Forward command controller for a set of position controlled joints (linear or angular).
 *
 * This class forwards the commanded positions down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The position commands to apply.
 */
class MMServoController : public controller_interface::ControllerInterface
{
public:
  MM_CONTROLLERS_PUBLIC
  MMServoController();

  MM_CONTROLLERS_PUBLIC 
  controller_interface::CallbackReturn on_init() override;
  
  MM_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration 
    command_interface_configuration() const override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  bool getMMIKResult(
    const mcdd::BaseState& base_state, const mcdd::ArmState& arm_state,
    const tf2::Transform& curr_pose, const tf2::Transform& goal_pose);
  bool updateArmCmd(
    mcdd::ArmData& arm_data, const geometry_msgs::msg::Twist& cmd, 
    uint8_t cmd_frame, const tf2::Transform base_offset_trans,
    const rclcpp::Time & time, const rclcpp::Duration& period, bool do_ik=true);
  bool updateBaseCmd(
    const geometry_msgs::msg::Twist& cmd, const uint8_t cmd_frame, 
    const rclcpp::Duration& period);
  bool calcWheelCmd(
    const rclcpp::Duration& period, int cnt=0);
  
  void armHalt(
    mcdd::ArmData& arm_data, const rclcpp::Duration & period);
  void baseHalt(
    const rclcpp::Duration & period);

  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT,
  };

  std::map<std::string, mcdd::HWInterfaces> arm_interfaces_;
  std::map<std::string, mcdd::HWInterfaces> wheel_interfaces_;

  std::string base_prefix_;
  bool subscriber_is_active_;
  std::map<std::string, bool> arm_is_halted_;

  using Params = mm_controllers::Params;
  using ParamListener = mm_controllers::ParamListener;
  // Parameters from ROS for joint_trajectory_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<mm_msgs::srv::IK::Response>> rt_mm_ik_res_ptr_;

  rclcpp::Subscription<CmdType>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Client<mm_msgs::srv::IK>::SharedPtr mm_ik_client_;

  mcdd::BaseData base_data_;
  std::map<std::string, mcdd::ArmData> arm_data_;
  FourWheelSteeringKinematics base_kinematics_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Transform base_moving_trans_;

  std::shared_ptr<mm_msgs::srv::IK::Response> last_mm_ik_res_;
  
  moveit::core::RobotStatePtr robot_state_;
  std::future<void> init_robot_state_thread_future_;
};

}  // namespace mm_controllers

#endif  // MM_CONTROLLERS__MM_SERVO_CONTROLLER_HPP_
