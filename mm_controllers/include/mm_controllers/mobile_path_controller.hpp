// Copyright (c) 2019 Intel Corporation
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

#ifndef MM_CONTROLLERS__MOBILE_PATH_CONTROLLER_HPP_
#define MM_CONTROLLERS__MOBILE_PATH_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <mutex>

#include <rclcpp_action/server.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav2_msgs/action/follow_path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "mm_controllers/visibility_control.h"
#include "mm_controllers/mm_data_define.hpp"
#include "mm_controllers/four_wheel_steering_kinematics.hpp"

#include "mm_controller_parameters.hpp"

namespace mm_controllers
{
namespace mcdd = mm_controllers::data_define;
class Trajectory;
// class ProgressChecker;
/**
 * @class mm_controllers::MobilePathController
 * @brief This class will generate the trajectory for base from given path,
 *  then make robot trace it.
 */
class MobilePathController : public controller_interface::ControllerInterface
{
public:
  /**
   * @brief Constructor for mm_controllers::MobilePathController
   * @param options Additional options to control creation of the node.
   */
  MM_CONTROLLERS_PUBLIC
  explicit MobilePathController();
  /**
   * @brief Destructor for mm_controllers::MobilePathController
   */
  MM_CONTROLLERS_PUBLIC
  ~MobilePathController();

protected:
  /**
   * @brief command_interface_configuration This controller requires the position command
   * interfaces for the controlled joints
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief command_interface_configuration This controller requires the position and velocity
   * state interfaces for the controlled joints
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configures controller parameters and member variables
   *
   * Configures controller plugin and costmap; Initialize odom subscriber,
   * velocity publisher and follow path action server.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize controller
   * plugin
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates member variables
   *
   * Activates controller, costmap, velocity publisher and follow path action
   * server
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates member variables
   *
   * Deactivates follow path action server, controller, costmap and velocity
   * publisher. Before calling deactivate state, velocity is being set to zero.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Calls clean up states and resets member variables.
   *
   * Controller and costmap clean up state is called, and resets rest of the
   * variables
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;

  using FollowPath = nav2_msgs::action::FollowPath;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowPath>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

  // Our action server implements the FollowPath action
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  RealtimeGoalHandleBuffer rt_active_goal_;  ///< Currently active action goal, if any.
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_{std::chrono::milliseconds(50)};

  /**
   * @brief FollowPath action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * Provides global path to controller received from action client. Twist
   * velocities for the robot are calculated and published using controller at
   * the specified rate till the goal is reached.
   * @throw nav2_core::PlannerException
   */
  MM_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Assigns path to controller
   * @param path Path received from action server
   */
  MM_CONTROLLERS_PUBLIC
  void setPlannerPath(const nav_msgs::msg::Path & path);

  /**
   * @brief Checks if goal is reached
   * @return true or false
   */
  MM_CONTROLLERS_PUBLIC
  bool isGoalReached(const mcdd::BaseData& base_data);

  // callbacks for action_server_
  MM_CONTROLLERS_PUBLIC
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowPath::Goal> goal);
  MM_CONTROLLERS_PUBLIC
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> goal_handle);
  MM_CONTROLLERS_PUBLIC
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> goal_handle);
  MM_CONTROLLERS_PUBLIC
  void preempt_active_goal();

  const std::vector<std::string> allowed_interface_types_ = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION,
    hardware_interface::HW_IF_EFFORT,
  };

  using Params = mm_controllers::Params;
  using ParamListener = mm_controllers::ParamListener;
  // Parameters from ROS for joint_trajectory_controller
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  std::shared_ptr<Trajectory> * traj_point_active_ptr_{nullptr};
  std::shared_ptr<Trajectory> traj_external_point_ptr_{nullptr};
  realtime_tools::RealtimeBuffer<std::shared_ptr<
    trajectory_msgs::msg::JointTrajectory>> traj_msg_external_point_ptr_;
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator
    running_point_iter_;

private:
  MM_CONTROLLERS_PUBLIC
  bool updateBaseCmd(const rclcpp::Duration& period);
  MM_CONTROLLERS_PUBLIC
  void computeVelocityCommands(
    const rclcpp::Time& time, const rclcpp::Duration& period, 
    const tf2::Transform& pose, const mcdd::BaseData& base_data,
    mcdd::BaseState& goal_state);

  bool steering_mode_{false};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2::Transform> map_trans_odom_delay_{nullptr};
  tf2::Transform map_trans_odom_;
  std::vector<std::string> wheel_names_;
  std::map<std::string, mcdd::HWInterfaces> wheel_interfaces_;
  FourWheelSteeringKinematics base_kinematics_;
  mcdd::BaseData base_data_;
  interpolation_methods::InterpolationMethod interpolation_method_{
    interpolation_methods::DEFAULT_INTERPOLATION};
};

}  // namespace mm_controllers

#endif  // MM_CONTROLLERS__MOBILE_PATH_CONTROLLER_HPP_
