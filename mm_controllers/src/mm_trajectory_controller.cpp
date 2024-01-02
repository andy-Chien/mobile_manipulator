// Copyright (c) 2021 ros2_control Development Team
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

#include <stddef.h>
#include <chrono>
#include <functional>
#include <memory>
#include <ostream>
#include <ratio>
#include <string>
#include <vector>

#include <angles/angles.h>
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mm_controllers/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/header.hpp"

#include "mm_controllers/mm_controller_utils.hpp"
#include "mm_controllers/mm_trajectory_controller.hpp"
namespace mm_controllers
{
MMTrajectoryController::MMTrajectoryController()
: controller_interface::ControllerInterface(), dof_(0), arm_dof_(0), base_dof_(0)
{
}

controller_interface::CallbackReturn MMTrajectoryController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    wheel_names_ = params_.wheel_names;
    base_kinematics_ = FourWheelSteeringKinematics();
    // Set interpolation method from string parameter
    interpolation_method_ = interpolation_methods::from_string(params_.interpolation_method);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MMTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    conf.names.reserve(arm_data_.joint_names.size() * arm_data_.command_interface_types.size() +
                       wheel_names_.size() * base_data_.swerve_command_interfaces.size() + 
                       wheel_names_.size() * base_data_.wheel_command_interfaces.size());
    for(const auto & joint_name : arm_data_.joint_names)
    {
      for(const auto & interface_type : arm_data_.command_interface_types)
      {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }
    for(const auto & wheel_name : wheel_names_)
    {
      for(const auto & interface_type : base_data_.swerve_command_interfaces)
      {
        conf.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[0] + "/" + interface_type);
      }
      for(const auto & interface_type : base_data_.wheel_command_interfaces)
      {
        conf.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[1] + "/" + interface_type);
      }
    }
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, 
      "Exception thrown during command_interface_configuration with message: %s \n", e.what());
    return conf;
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MMTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    conf.names.reserve(arm_data_.joint_names.size() * arm_data_.state_interface_types.size() +
                       wheel_names_.size() * base_data_.swerve_state_interfaces.size() + 
                       wheel_names_.size() * base_data_.wheel_state_interfaces.size());
    for(const auto & joint_name : arm_data_.joint_names)
    {
      for(const auto & interface_type : arm_data_.state_interface_types)
      {
        conf.names.push_back(joint_name + "/" + interface_type);
      }
    }
    for(const auto & wheel_name : wheel_names_)
    {
      for(const auto & interface_type : base_data_.swerve_state_interfaces)
      {
        conf.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[0] + "/" + interface_type);
      }
      for(const auto & interface_type : base_data_.wheel_state_interfaces)
      {
        conf.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[1] + "/" + interface_type);
      }
    }
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, 
      "Exception thrown during state_interface_configuration with message: %s \n", e.what());
    return conf;
  }
  return conf;
}

controller_interface::return_type MMTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  auto compute_error_for_joint = [&](
                                   JointTrajectoryPoint & error, int index,
                                   const JointTrajectoryPoint & current,
                                   const JointTrajectoryPoint & desired)
  {
    // error defined as the difference between current and desired
    // index 0, 1 are base x, y
    if(index < 2){
      error.positions[index] = desired.positions[index] - current.positions[index];
    }else{
      error.positions[index] =
        angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    }
    if (has_velocity_state_interface_ && has_velocity_command_interface_)
    {
      error.velocities[index] = desired.velocities[index] - current.velocities[index];
    }
    if (has_acceleration_state_interface_ && has_acceleration_command_interface_)
    {
      error.accelerations[index] = desired.accelerations[index] - current.accelerations[index];
    }
  };

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_hardware(state_current_); //only for arm
  if(!updateBaseState(time, period)){
    return controller_interface::return_type::ERROR;
  }
  const tf2::Transform map_trans_odom_delay = 
    (map_trans_odom_delay_) ? *map_trans_odom_delay_ : tf2::Transform::getIdentity();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
    running_point_iter_ = traj_external_point_ptr_->begin();
  }

  if(!traj_point_active_ptr_ || (*traj_point_active_ptr_)->is_executed_already() || 
     !(*traj_point_active_ptr_)->has_trajectory_msg() || 
     !((*traj_point_active_ptr_)->number_of_points() > 0))
  {
    state_desired_ = last_commanded_state_;
    arm_data_.goal_state.jnt_pos = std::vector<double>(
      state_desired_.positions.begin() + base_dof_, state_desired_.positions.end());
    mcu::armHalt(period, arm_interfaces_, arm_data_, 
      has_position_command_interface_,
      has_velocity_command_interface_,
      has_acceleration_command_interface_,
      has_effort_command_interface_
    );
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    mcu::baseStateToJointState(base_data_.next_state, map_trans_base_, state_desired_);
    for (size_t index = 0; index < dof_; ++index)
    {
      compute_error_for_joint(state_error_, index, state_current_, state_desired_);
    }
    last_commanded_state_ = state_desired_;
    publish_state(state_desired_, state_current_, state_error_);
    return controller_interface::return_type::OK;
  }

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && (*traj_point_active_ptr_)->has_trajectory_msg())
  {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      first_sample = true;
      if (params_.open_loop_control)
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, last_commanded_state_);
      }
      else
      {
        (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current_);
      }
    }

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
      (*traj_point_active_ptr_)
        ->sample(time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);

    if (valid_point)
    {
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      double time_difference = 0.0;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();

      auto dist_to_curr_point = [this](std::vector<double> pos){
        const double dist = std::hypot(
          pos.at(0) - map_trans_base_.getOrigin()[0],
          pos.at(1) - map_trans_base_.getOrigin()[1]);
        const double ang_dist = fabs(angles::shortest_angular_distance(
          pos.at(2), mcu::quatToAngle(map_trans_base_.getRotation())));
        return std::tuple<double, double>({dist, ang_dist});
      };
      
      auto traj_back = traj_external_point_ptr_->end() - 1;
      if(running_point_iter_ < end_segment_itr && running_point_iter_ < traj_back)
      {
        const auto [pos_dist, ang_dist] = 
          dist_to_curr_point(running_point_iter_->positions);
        const auto [pos_dist_next, ang_dist_next] = 
          dist_to_curr_point((running_point_iter_ + 1)->positions);
        const auto& dtst = default_tolerances_.state_tolerance;
        if((dtst[0].position > 0.0 && pos_dist < dtst[0].position && 
            dtst[2].position > 0.0 && ang_dist < dtst[2].position) ||
            (ang_dist_next < ang_dist && pos_dist_next < pos_dist))
        {
          running_point_iter_ += 1;
        }
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index)
      {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        if (
          (before_last_point || first_sample) &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.state_tolerance[index], true))
        {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (
          !before_last_point &&
          !check_state_tolerance_per_joint(
            state_error_, index, default_tolerances_.goal_state_tolerance[index], false))
        {
          outside_goal_tolerance = true;

          if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_tolerance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            time_difference = get_node()->now().seconds() - traj_end.seconds();

            if (time_difference > default_tolerances_.goal_time_tolerance)
            {
              within_goal_time = false;
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time)
      {
        if (use_closed_loop_pid_adapter_)
        {
          // Update PIDs // only for arm joints
          for (auto i = 0ul; i < arm_dof_; ++i)
          {
            tmp_command_[i] = (state_desired_.velocities[i + base_dof_] * ff_velocity_scale_[i]) +
                              pids_[i]->computeCommand(
                                state_desired_.positions[i + base_dof_] - state_current_.positions[i + base_dof_],
                                state_desired_.velocities[i + base_dof_] - state_current_.velocities[i + base_dof_],
                                (uint64_t)period.nanoseconds());
          }
        }

        if(end_segment_itr > running_point_iter_)
        {
          RCLCPP_INFO(get_node()->get_logger(),
            "Behind the planned trajectory! %d", 
            int(end_segment_itr - running_point_iter_));
        }

        auto get_partial_state = [&](const trajectory_msgs::msg::JointTrajectoryPoint& state,
                                  size_t start, size_t end){
          trajectory_msgs::msg::JointTrajectoryPoint jtp;
          jtp.positions = std::vector<double>(
            state.positions.begin() + start, state.positions.begin() + end);
          jtp.velocities = std::vector<double>(
            state.velocities.begin() + start, state.velocities.begin() + end);
          jtp.accelerations = std::vector<double>(
            state.accelerations.begin() + start, state.accelerations.begin() + end);
          return jtp;
        };

        const bool not_halting = 
          fabs(state_desired_.velocities[0]) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT ||
          fabs(state_desired_.velocities[1]) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT ||
          fabs(state_desired_.velocities[2]) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT;

        if(!steering_mode_)
        {
          auto base_state_desired = get_partial_state(state_desired_, 0, base_dof_);
          auto base_state_refer = get_partial_state(last_commanded_state_, 0, base_dof_);
          const auto& base_state_curr = get_partial_state(state_current_, 0, base_dof_);
          const auto& base_state_goal = get_partial_state(*running_point_iter_, 0, base_dof_);
          if(not_halting){
            base_state_refer.positions = base_state_curr.positions;
            base_state_refer.positions[2] = base_state_goal.positions[2] - state_error_.positions[2];
          }
          const double t_x = mcu::calcMovingTime(
            base_state_goal.positions[0] - base_state_refer.positions[0], 
            base_data_.dir_vel_max, base_data_.dir_acc_max, 
            base_state_refer.velocities[0], base_state_goal.velocities[0]
          );
          const double t_y = mcu::calcMovingTime(
            base_state_goal.positions[1] - base_state_refer.positions[1],
            base_data_.dir_vel_max, base_data_.dir_acc_max, 
            base_state_refer.velocities[1], base_state_goal.velocities[1]
          );
          const double t_z = mcu::calcMovingTime(
            base_state_goal.positions[2] - base_state_refer.positions[2], 
            base_data_.ang_vel_max, base_data_.ang_acc_max, 
            base_state_refer.velocities[2], base_state_goal.velocities[2]
          );
          // 1.2 here because time calc doesn't consider jerk,                                                        
          // so the actual running time is longer
          double t_max = std::max(std::max(t_x, t_y), t_z) * 1.2;
          rclcpp::Duration tt = rclcpp::Duration(
            int32_t(t_max), uint32_t((t_max*1e9)) % uint32_t(1e9));

          if(tt < period){
            base_state_desired = get_partial_state(*running_point_iter_, 0, base_dof_);
          }else{
            (*traj_point_active_ptr_)->interpolate_between_points(time, base_state_refer, 
              time + tt, base_state_goal, time + period, base_state_desired);
          }
          for(size_t i = 0; i < base_dof_; i++)
          {
            state_desired_.positions[i] =  base_state_desired.positions[i];
            state_desired_.velocities[i] = base_state_desired.velocities[i];
            state_desired_.accelerations[i] = base_state_desired.accelerations[i];
          }
          mcu::jointStateToBaseState(
            state_desired_, map_trans_odom_delay, map_trans_base_, base_data_.goal_state);
        }

        if(updateBaseCmd(period)){
          mcu::setBaseCmd(wheel_interfaces_, base_data_);
        }
        mcu::baseStateToJointState(base_data_.next_state, map_trans_base_, state_desired_);

        if(steering_mode_ && arm_data_.is_halted)
        {
          arm_data_.goal_state.jnt_pos = std::vector<double>(
            last_commanded_state_.positions.begin() + base_dof_, 
            last_commanded_state_.positions.end()
          );
          arm_data_.goal_state.jnt_vel = std::vector<double>(
            last_commanded_state_.velocities.begin() + base_dof_, 
            last_commanded_state_.velocities.end()
          );

          mcu::armHalt(period, arm_interfaces_, arm_data_);
          (*traj_point_active_ptr_)->set_trajectory_start_time(
            (*traj_point_active_ptr_)->get_trajectory_start_time() + period
          );
          
          for (size_t index = 0; index < dof_; ++index)
          {
            if(index >= base_dof_)
            {
              state_desired_.positions[index] = last_commanded_state_.positions[index];
              state_desired_.velocities[index] = last_commanded_state_.velocities[index];
              state_desired_.accelerations[index] = last_commanded_state_.accelerations[index];
            }
            compute_error_for_joint(state_error_, index, state_current_, state_desired_);
          }
          publish_state(state_desired_, state_current_, state_error_);
          last_commanded_state_ = state_desired_;
          return controller_interface::return_type::OK;
        }

        arm_data_.goal_state.jnt_pos = std::vector<double>(
          state_desired_.positions.begin() + base_dof_, state_desired_.positions.end());
        if (use_closed_loop_pid_adapter_){
          arm_data_.goal_state.jnt_vel = tmp_command_;
        }else{
          arm_data_.goal_state.jnt_vel = std::vector<double>(
            state_desired_.velocities.begin() + base_dof_, state_desired_.velocities.end());
        }

        mcu::ensureArmCmdLimit(period, arm_data_);
        mcu::setArmCmd(
          arm_interfaces_,
          arm_data_,
          has_position_command_interface_,
          has_velocity_command_interface_,
          has_acceleration_command_interface_,
          has_effort_command_interface_
        );

        // store the previous command. Used in open-loop control mode
        last_commanded_state_ = state_desired_;
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = joint_names_;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving)
        {
          set_hold_position();
          auto result = std::make_shared<FollowJTrajAction::Result>();

          RCLCPP_WARN(get_node()->get_logger(), "Aborted due to state tolerance violation");
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

          // check goal tolerance
        }
        else if (!before_last_point)
        {
          if (!outside_goal_tolerance)
          {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            (*traj_point_active_ptr_)->set_executed();
            RCLCPP_INFO(get_node()->get_logger(), "Goal reached, success!");
          }
          else if (!within_goal_time)
          {
            set_hold_position();
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            RCLCPP_WARN(
              get_node()->get_logger(), 
              "Aborted due goal_time_tolerance exceeding by %f seconds",
              time_difference);
          }
          // else, run another cycle while waiting for outside_goal_tolerance
          // to be satisfied or violated within the goal_time_tolerance
        }
      }
      else if (tolerance_violated_while_moving)
      {
        set_hold_position();
        RCLCPP_ERROR(get_node()->get_logger(), 
          "Holding position due to state tolerance violation");
      }
    }
  }

  publish_state(state_desired_, state_current_, state_error_);
  return controller_interface::return_type::OK;
}

void MMTrajectoryController::read_state_from_hardware(JointTrajectoryPoint & state)
{
  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < arm_dof_; ++index)
    {
      trajectory_point_interface[index + base_dof_] = joint_interface[index].get().get_value();
    }
  };

  // Assign values from the hardware
  // Position states always exist
  assign_point_from_interface(state.positions, arm_interfaces_.state[0]);
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    assign_point_from_interface(state.velocities, arm_interfaces_.state[1]);
    // Acceleration is used only in combination with velocity
    if (has_acceleration_state_interface_)
    {
      assign_point_from_interface(state.accelerations, arm_interfaces_.state[2]);
    }
  }
}

bool MMTrajectoryController::read_state_from_command_interfaces(JointTrajectoryPoint & state)
{
  bool has_values = true;

  auto assign_point_from_interface =
    [&](std::vector<double> & trajectory_point_interface, const auto & joint_interface)
  {
    for (size_t index = 0; index < arm_dof_; ++index)
    {
      trajectory_point_interface[index + base_dof_] = joint_interface[index].get().get_value();
    }
  };

  auto interface_has_values = [](const auto & joint_interface)
  {
    return std::find_if(
             joint_interface.begin(), joint_interface.end(),
             [](const auto & interface)
             { return std::isnan(interface.get().get_value()); }) == joint_interface.end();
  };

  // Assign values from the command interfaces as state. Therefore needs check for both.
  // Position state interface has to exist always
  if (has_position_command_interface_ && interface_has_values(arm_interfaces_.command[0]))
  {
    assign_point_from_interface(state.positions, arm_interfaces_.command[0]);
  }
  else
  {
    state.positions.clear();
    has_values = false;
  }
  // velocity and acceleration states are optional
  if (has_velocity_state_interface_)
  {
    if (has_velocity_command_interface_ && interface_has_values(arm_interfaces_.command[1]))
    {
      assign_point_from_interface(state.velocities, arm_interfaces_.command[1]);
    }
    else
    {
      state.velocities.clear();
      has_values = false;
    }
  }
  else
  {
    state.velocities.clear();
  }
  // Acceleration is used only in combination with velocity
  if (has_acceleration_state_interface_)
  {
    if (has_acceleration_command_interface_ && interface_has_values(arm_interfaces_.command[2]))
    {
      assign_point_from_interface(state.accelerations, arm_interfaces_.command[2]);
    }
    else
    {
      state.accelerations.clear();
      has_values = false;
    }
  }
  else
  {
    state.accelerations.clear();
  }

  return has_values;
}

controller_interface::CallbackReturn MMTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();
  joint_names_ = params_.base_joints;
  joint_names_.insert(joint_names_.end(), 
    params_.arm_joints.begin(), params_.arm_joints.end());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  map_trans_odom_ = tf2::Transform();
  map_trans_odom_.setIdentity();
  map_trans_base_ = tf2::Transform();
  map_trans_base_.setIdentity();
  
  // Check if the DoF has changed
  if ((dof_ > 0) && (joint_names_.size() != dof_))
  {
    RCLCPP_ERROR(
      logger,
      "The MMTrajectoryController does not support restarting with a different number of DOF");
    // TODO(andyz): update vector lengths if num. joints did change and re-initialize them so we
    // can continue
    return CallbackReturn::FAILURE;
  }

  dof_ = joint_names_.size();
  arm_dof_ = params_.arm_joints.size();
  base_dof_ = params_.base_joints.size();

  // TODO(destogl): why is this here? Add comment or move
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  if (joint_names_.empty())
  {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  mcu::initArmData(params_, arm_data_);
  mcu::initBaseData(params_, base_data_);
  base_kinematics_.configuration(base_data_);

  for(const auto& wheel_name : wheel_names_)
  {
    mcdd::HWInterfaces interfaces;
    interfaces.command.resize(allowed_interface_types_.size());
    interfaces.state.resize(allowed_interface_types_.size());
    wheel_interfaces_.insert(std::pair<std::string, mcdd::HWInterfaces>(wheel_name, interfaces));
  }

  if (params_.arm_params.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  arm_interfaces_.command.resize(allowed_interface_types_.size());

  has_position_command_interface_ = contains_interface_type(
    params_.arm_params.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ = contains_interface_type(
    params_.arm_params.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_command_interface_ = contains_interface_type(
    params_.arm_params.command_interfaces, hardware_interface::HW_IF_ACCELERATION);
  has_effort_command_interface_ = contains_interface_type(
    params_.arm_params.command_interfaces, hardware_interface::HW_IF_EFFORT);

  // if there is only velocity or if there is effort command interface
  // then use also PID adapter
  // TODO(andyc): use pid on partail joint
  use_closed_loop_pid_adapter_ =
    (has_velocity_command_interface_ && params_.arm_params.command_interfaces.size() == 1) ||
    has_effort_command_interface_;
  // TODO(andyc): use pid on partail joint
  if (use_closed_loop_pid_adapter_)
  {
    pids_.resize(arm_dof_);
    ff_velocity_scale_.resize(arm_dof_);
    tmp_command_.resize(arm_dof_, 0.0);

    // Init PID gains from ROS parameters
    for (size_t i = 0; i < arm_dof_; ++i)
    {
      const auto & gains = params_.arm_params.gains.arm_joints_map.at(params_.arm_joints[i]);
      pids_[i] = std::make_shared<control_toolbox::Pid>(
        gains.p, gains.i, gains.d, gains.i_clamp, -gains.i_clamp);

      // TODO(destogl): remove this in ROS2 Iron
      // Check deprecated style for "ff_velocity_scale" parameter definition.
      if (gains.ff_velocity_scale == 0.0)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "'ff_velocity_scale' parameters is not defined under 'gains.<joint_name>.' structure. "
          "Maybe you are using deprecated format 'ff_velocity_scale/<joint_name>'!");

        ff_velocity_scale_[i] = auto_declare<double>(
          "ff_velocity_scale/" + params_.arm_joints[i], 0.0);
      }
      else
      {
        ff_velocity_scale_[i] = gains.ff_velocity_scale;
      }
    }
  }

  if (params_.arm_params.state_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  // Note: 'effort' storage is also here, but never used. Still, for this is OK.
  arm_interfaces_.state.resize(allowed_interface_types_.size());

  has_position_state_interface_ = contains_interface_type(
    params_.arm_params.state_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ = contains_interface_type(
    params_.arm_params.state_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_acceleration_state_interface_ = contains_interface_type(
    params_.arm_params.state_interfaces, hardware_interface::HW_IF_ACCELERATION);

  // Validation of combinations of state and velocity together have to be done
  // here because the parameter validators only deal with each parameter
  // separately.
  if (
    has_velocity_command_interface_ && params_.arm_params.command_interfaces.size() == 1 &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'velocity' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  // effort is always used alone so no need for size check
  if (
    has_effort_command_interface_ &&
    (!has_velocity_state_interface_ || !has_position_state_interface_))
  {
    RCLCPP_ERROR(
      logger,
      "'effort' command interface can only be used alone if 'velocity' and "
      "'position' state interfaces are present");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "Command interfaces are [%s] and state interfaces are [%s].",
    get_interface_list(params_.arm_params.command_interfaces).c_str(),
    get_interface_list(params_.arm_params.state_interfaces).c_str());

  default_tolerances_ = get_segment_tolerances(params_);

  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());

  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&MMTrajectoryController::topic_callback, this, std::placeholders::_1));

  // State publisher
  RCLCPP_INFO(logger, 
    "Controller state will be published at %.2f Hz.", params_.state_publish_rate);
  if (params_.state_publish_rate > 0.0)
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(1.0 / params_.state_publish_rate);
  }
  else
  {
    state_publisher_period_ = rclcpp::Duration::from_seconds(0.0);
  }

  publisher_ =
    get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<StatePublisher>(publisher_);

  state_publisher_->lock();
  state_publisher_->msg_.joint_names = joint_names_;
  state_publisher_->msg_.desired.positions.resize(dof_);
  state_publisher_->msg_.desired.velocities.resize(dof_);
  state_publisher_->msg_.desired.accelerations.resize(dof_);
  state_publisher_->msg_.actual.positions.resize(dof_);
  state_publisher_->msg_.error.positions.resize(dof_);
  if (has_velocity_state_interface_)
  {
    state_publisher_->msg_.actual.velocities.resize(dof_);
    state_publisher_->msg_.error.velocities.resize(dof_);
  }
  if (has_acceleration_state_interface_)
  {
    state_publisher_->msg_.actual.accelerations.resize(dof_);
    state_publisher_->msg_.error.accelerations.resize(dof_);
  }
  state_publisher_->unlock();

  last_state_publish_time_ = get_node()->now();

  // action server configuration
  if (params_.allow_partial_joints_goal)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  RCLCPP_INFO(logger, 
    "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_trajectory",
    std::bind(&MMTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&MMTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&MMTrajectoryController::goal_accepted_callback, this, _1));

  odom_broadcaster_ = std::make_unique<
    tf2_ros::TransformBroadcaster>(*this->get_node());
  odom_pub_ = this->get_node()->create_publisher<
    nav_msgs::msg::Odometry>(mcdd::DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

  resize_joint_trajectory_point(state_current_, dof_);
  resize_joint_trajectory_point(state_desired_, dof_);
  resize_joint_trajectory_point(state_error_, dof_);
  resize_joint_trajectory_point(last_commanded_state_, dof_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  // order all joints in the storage
  for (const auto & interface : params_.arm_params.command_interfaces)
  {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, 
      params_.arm_joints, interface, arm_interfaces_.command[index]))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Expected %zu '%s' command interfaces, got %zu.", dof_,
        interface.c_str(), arm_interfaces_.command[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : params_.arm_params.state_interfaces)
  {
    auto it = std::find(
      allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = std::distance(allowed_interface_types_.begin(), it);
    if (!controller_interface::get_ordered_interfaces(state_interfaces_,
      params_.arm_joints, interface, arm_interfaces_.state[index]))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Expected %zu '%s' state interfaces, got %zu.", dof_,
        interface.c_str(), arm_interfaces_.state[index].size());
      return CallbackReturn::ERROR;
    }
  }

  for(const auto & wheel_name : wheel_names_)
  {
    std::string swerve_joint_name = base_data_.wheel_data.at(wheel_name)->joints_name[0];
    std::string wheel_joint_name = base_data_.wheel_data.at(wheel_name)->joints_name[1];
    for(size_t index = 0; index < allowed_interface_types_.size(); index++)
    {
      const auto & interface = allowed_interface_types_[index];
      std::vector<std::string> joint_names;
      if(std::find(base_data_.swerve_command_interfaces.begin(), 
        base_data_.swerve_command_interfaces.end(), interface)
        != base_data_.swerve_command_interfaces.end())
      {
        joint_names.push_back(swerve_joint_name);
      }
      if(std::find(base_data_.wheel_command_interfaces.begin(),
        base_data_.wheel_command_interfaces.end(), interface)
        != base_data_.wheel_command_interfaces.end())
      {
        joint_names.push_back(wheel_joint_name);
      }
      if(joint_names.size() == 0)
      {
        continue;
      }
      if(!controller_interface::get_ordered_interfaces(command_interfaces_,
        joint_names, interface, wheel_interfaces_.at(wheel_name).command[index]))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' command interfaces, got %zu.",
          interface.c_str(), wheel_interfaces_.at(wheel_name).command[index].size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    for(size_t index = 0; index < allowed_interface_types_.size(); index++)
    {
      const auto & interface = allowed_interface_types_[index];
      std::vector<std::string> joint_names;
      if(std::find(base_data_.swerve_state_interfaces.begin(), 
        base_data_.swerve_state_interfaces.end(), interface)
        != base_data_.swerve_state_interfaces.end())
      {
        joint_names.push_back(swerve_joint_name);
      }
      if(std::find(base_data_.wheel_state_interfaces.begin(),
        base_data_.wheel_state_interfaces.end(), interface)
        != base_data_.wheel_state_interfaces.end())
      {
        joint_names.push_back(wheel_joint_name);
      }
      if(joint_names.size() == 0)
      {
        continue;
      }
      if(!controller_interface::get_ordered_interfaces(state_interfaces_,
        joint_names, interface, wheel_interfaces_[wheel_name].state[index]))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' state interfaces, got %zu.",
          interface.c_str(), wheel_interfaces_[wheel_name].state[index].size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Store 'home' pose
  traj_msg_home_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg_home_ptr_->header.stamp.sec = 0;
  traj_msg_home_ptr_->header.stamp.nanosec = 0;
  traj_msg_home_ptr_->points.resize(1);
  traj_msg_home_ptr_->points[0].time_from_start.sec = 0;
  traj_msg_home_ptr_->points[0].time_from_start.nanosec = 50000000;
  traj_msg_home_ptr_->points[0].positions.resize(arm_interfaces_.state[0].size());
  for (size_t index = 0; index < arm_interfaces_.state[0].size(); ++index)
  {
    traj_msg_home_ptr_->points[0].positions[index] =
      arm_interfaces_.state[0][index].get().get_value();
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_home_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;
  traj_point_active_ptr_ = &traj_external_point_ptr_;
  last_state_publish_time_ = get_node()->now();

  // Initialize current state storage if hardware state has tracking offset
  read_state_from_hardware(state_current_);

  geometry_msgs::msg::TransformStamped odom_trans_base_msgs;
  if(mcu::getTf(tf_buffer_, "odom", "mobile_base_footprint", odom_trans_base_msgs, rclcpp::Duration(500ms)))
  {
    tf2::Transform odom_trans_base;
    mcu::msgToTf(odom_trans_base_msgs.transform, odom_trans_base);
    auto& t = odom_trans_base.getOrigin();
    double angle = mcu::quatToAngle(odom_trans_base.getRotation());
    base_data_.curr_state.position[0] = t[0];
    base_data_.curr_state.position[1] = t[1];
    base_data_.curr_state.rotation = angle;
    base_data_.last_state.position[0] = t[0];
    base_data_.last_state.position[1] = t[1];
    base_data_.last_state.rotation = angle;
  }

  if(!updateBaseState(rclcpp::Time(0), rclcpp::Duration(10ms))){
    return CallbackReturn::ERROR;
  }
  state_desired_ = state_current_;
  last_commanded_state_ = state_current_;

  arm_data_.curr_state.jnt_pos = std::vector<double>(
    state_current_.positions.begin() + base_dof_, state_current_.positions.end());
  arm_data_.curr_state.jnt_vel = std::vector<double>(
    state_current_.velocities.begin() + base_dof_, state_current_.velocities.end());
  arm_data_.curr_state.jnt_acc = std::vector<double>(
    state_current_.accelerations.begin() + base_dof_, state_current_.accelerations.end());
  arm_data_.curr_state.pose = std::vector<double>(
    7, std::numeric_limits<double>::infinity());

  arm_data_.last_state = arm_data_.curr_state;
  arm_data_.next_state = arm_data_.curr_state;
  arm_data_.goal_state = arm_data_.curr_state;

  // Handle restart of controller by reading from commands if
  // those are not nan
  trajectory_msgs::msg::JointTrajectoryPoint state;
  resize_joint_trajectory_point(state, dof_);
  if (read_state_from_command_interfaces(state))
  {
    mcu::baseStateToJointState(base_data_.last_state, map_trans_base_, state);
    state_current_ = state;
    state_desired_ = state;
    last_commanded_state_ = state;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // TODO(anyone): How to halt when using effort commands?
  try
  {
    for (size_t index = 0; index < arm_dof_; ++index)
    {
      if (has_position_command_interface_)
      {
        arm_interfaces_.command[0][index].get().set_value(
          arm_interfaces_.command[0][index].get().get_value());
      }

      if (has_velocity_command_interface_)
      {
        arm_interfaces_.command[1][index].get().set_value(0.0);
      }

      if (has_effort_command_interface_)
      {
        arm_interfaces_.command[3][index].get().set_value(0.0);
      }
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "fail to set halt value: %s \n", e.what());
  }
  

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    arm_interfaces_.command[index].clear();
    arm_interfaces_.state[index].clear();
    for(const auto& wheel_name : wheel_names_)
    {
      wheel_interfaces_.at(wheel_name).command[index].clear();
      wheel_interfaces_.at(wheel_name).state[index].clear();
    }
  }
  release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // go home
  if (traj_home_point_ptr_ != nullptr)
  {
    traj_home_point_ptr_->update(traj_msg_home_ptr_);
    traj_point_active_ptr_ = &traj_home_point_ptr_;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMTrajectoryController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool MMTrajectoryController::reset()
{
  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();

  for (const auto & pid : pids_)
  {
    if (pid)
    {
      pid->reset();
    }
  }

  // iterator has no default value
  // prev_traj_point_ptr_;
  traj_point_active_ptr_ = nullptr;
  traj_external_point_ptr_.reset();
  traj_home_point_ptr_.reset();
  traj_msg_home_ptr_.reset();

  return true;
}

controller_interface::CallbackReturn MMTrajectoryController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  // TODO(karsten1987): what to do?

  return CallbackReturn::SUCCESS;
}

void MMTrajectoryController::publish_state(
  const JointTrajectoryPoint & desired_state, const JointTrajectoryPoint & current_state,
  const JointTrajectoryPoint & state_error)
{
  if (state_publisher_period_.seconds() <= 0.0)
  {
    return;
  }

  if (get_node()->now() < (last_state_publish_time_ + state_publisher_period_))
  {
    return;
  }

  if (state_publisher_ && state_publisher_->trylock())
  {
    last_state_publish_time_ = get_node()->now();
    state_publisher_->msg_.header.stamp = last_state_publish_time_;
    state_publisher_->msg_.desired.positions = desired_state.positions;
    state_publisher_->msg_.desired.velocities = desired_state.velocities;
    state_publisher_->msg_.desired.accelerations = desired_state.accelerations;
    state_publisher_->msg_.actual.positions = current_state.positions;
    state_publisher_->msg_.error.positions = state_error.positions;
    if (has_velocity_state_interface_)
    {
      state_publisher_->msg_.actual.velocities = current_state.velocities;
      state_publisher_->msg_.error.velocities = state_error.velocities;
    }
    if (has_acceleration_state_interface_)
    {
      state_publisher_->msg_.actual.accelerations = current_state.accelerations;
      state_publisher_->msg_.error.accelerations = state_error.accelerations;
    }

    state_publisher_->unlockAndPublish();
  }
}

void MMTrajectoryController::topic_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  if (!validate_trajectory_msg(*msg))
  {
    return;
  }
  // http://wiki.ros.org/joint_trajectory_controller/UnderstandingTrajectoryReplacement
  // always replace old msg with new one for now
  if (subscriber_is_active_)
  {
    add_new_trajectory_msg(msg);
  }
};

rclcpp_action::GoalResponse MMTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MMTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    // Controller uptime
    // Enter hold current position mode
    set_hold_position();

    RCLCPP_DEBUG(get_node()->get_logger(),
      "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MMTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(
        goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void MMTrajectoryController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // joint names in the goal are a subset of existing joints, as checked in goal_callback
  // so if the size matches, the goal contains all controller joints
  if (dof_ == trajectory_msg->joint_names.size())
  {
    return;
  }

  trajectory_msg->joint_names.reserve(dof_);
  const auto get_pos_from_cmd_interface =
    [&](const size_t index){
      return arm_interfaces_.command[0][index - base_dof_].get().get_value();
    };

  const auto get_pos_from_state_interface =
    [&](const size_t index){
      return arm_interfaces_.state[0][index - base_dof_].get().get_value();
    };

  for (size_t index = 0; index < dof_; ++index)
  {
    {
      if (
        std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          joint_names_[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(joint_names_[index]);

      for (auto & it : trajectory_msg->points)
      {
        // Assume hold position with 0 velocity and acceleration for missing joints
        if (!it.positions.empty())
        {
          if(index < base_dof_)
          {
            if(!std::isnan(last_commanded_state_.positions.at(index)))
            {
              it.positions.push_back(last_commanded_state_.positions.at(index));
            }
            else
            {
              it.positions.push_back(state_current_.positions.at(index));
            }
          }
          else
          {
            if (
              has_position_command_interface_ &&
              !std::isnan(get_pos_from_cmd_interface(index)))
            {
              // copy last command if cmd interface exists
              it.positions.push_back(last_commanded_state_.positions.at(index));
            }
            else if (has_position_state_interface_)
            {
              // copy current state if state interface exists
              it.positions.push_back(get_pos_from_state_interface(index));
            }
          }
        }
        if (!it.velocities.empty())
        {
          it.velocities.push_back(0.0);
        }
        if (!it.accelerations.empty())
        {
          it.accelerations.push_back(0.0);
        }
        if (!it.effort.empty())
        {
          it.effort.push_back(0.0);
        }
      }
    }
  }
}

void MMTrajectoryController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg)
{
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, joint_names_);
  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & mapping) -> std::vector<double>
  {
    if (to_remap.empty())
    {
      return to_remap;
    }
    if (to_remap.size() != mapping.size())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
      return to_remap;
    }
    std::vector<double> output;
    output.resize(mapping.size(), 0.0);
    for (size_t index = 0; index < mapping.size(); ++index)
    {
      auto map_index = mapping[index];
      output[map_index] = to_remap[index];
    }
    return output;
  };

  for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
  {
    trajectory_msg->points[index].positions =
      remap(trajectory_msg->points[index].positions, mapping_vector);

    trajectory_msg->points[index].velocities =
      remap(trajectory_msg->points[index].velocities, mapping_vector);

    trajectory_msg->points[index].accelerations =
      remap(trajectory_msg->points[index].accelerations, mapping_vector);

    trajectory_msg->points[index].effort =
      remap(trajectory_msg->points[index].effort, mapping_vector);
  }
}

bool MMTrajectoryController::validate_trajectory_point_field(
  size_t joint_names_size, const std::vector<double> & vector_field,
  const std::string & string_for_vector_field, size_t i, bool allow_empty) const
{
  if (allow_empty && vector_field.empty())
  {
    return true;
  }
  if (joint_names_size != vector_field.size())
  {
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Mismatch between joint_names (%zu) and %s (%zu) at point #%zu.",
      joint_names_size, string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

bool MMTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!params_.allow_partial_joints_goal)
  {
    if (trajectory.joint_names.size() != dof_)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored. (Andy) This looks useless and doesn't make sense, 
  // end time should only be the sum of start time and the time_from_start of the last point.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto trajectory_end_time = trajectory_start_time;
    for (const auto & p : trajectory.points)
    {
      trajectory_end_time += p.time_from_start;
    }
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non zero time start time (%f) that ends on the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(joint_names_.begin(), joint_names_.end(), incoming_joint_name);
    if (it == joint_names_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    if ((i > 0) && (rclcpp::Duration(
      trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    // This currently supports only position, velocity and acceleration inputs
    if (params_.allow_integration_in_goal_trajectories)
    {
      const bool all_empty = points[i].positions.empty() && points[i].velocities.empty() &&
                             points[i].accelerations.empty();
      const bool position_error =
        !points[i].positions.empty() &&
        !validate_trajectory_point_field(
          joint_count, points[i].positions, "positions", i, false);
      const bool velocity_error =
        !points[i].velocities.empty() &&
        !validate_trajectory_point_field(
          joint_count, points[i].velocities, "velocities", i, false);
      const bool acceleration_error =
        !points[i].accelerations.empty() &&
        !validate_trajectory_point_field(
          joint_count, points[i].accelerations, "accelerations", i, false);
      if (all_empty || position_error || velocity_error || acceleration_error)
      {
        return false;
      }
    }
    else if (
      !validate_trajectory_point_field(
        joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(
        joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].accelerations, "accelerations", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].effort, "effort", i, true))
    {
      return false;
    }
  }
  return true;
}

void MMTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void MMTrajectoryController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    set_hold_position();
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

void MMTrajectoryController::set_hold_position()
{
  trajectory_msgs::msg::JointTrajectory empty_msg;
  empty_msg.header.stamp = rclcpp::Time(0);

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>(empty_msg);
  add_new_trajectory_msg(traj_msg);
}

bool MMTrajectoryController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

void MMTrajectoryController::resize_joint_trajectory_point(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
{
  point.positions.resize(size, 0.0);
  point.velocities.resize(size, 0.0);
  point.accelerations.resize(size, 0.0);
}

bool MMTrajectoryController::updateBaseState(
  const rclcpp::Time & time, const rclcpp::Duration& period)
{
  // TODO(andyc): interfaces reader only read pos for steering joint 
  // and velocity for driving joint, we should all types of interface
  for(auto const& [wheel_name, interfaces] : wheel_interfaces_)
  {
    mcdd::WheelData* wheel_data = base_data_.wheel_data[wheel_name];
    mcdd::WheelState& cs = wheel_data->curr_state;
    mcdd::WheelState& ls = wheel_data->last_state;
    cs.st_pos = interfaces.state[0][0].get().get_value();
    cs.dr_vel = interfaces.state[1][1].get().get_value();
    cs.direction[0] = cos(cs.st_pos);
    cs.direction[1] = sin(cs.st_pos);
    if(fabs(ls.dr_vel) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
    {
      cs.direction[0] *= std::copysign(1.0, ls.dr_vel);
      cs.direction[1] *= std::copysign(1.0, ls.dr_vel);
    }
    else if((cs.direction[0] * ls.direction[0] + cs.direction[1] * ls.direction[1]) < 0)
    {
      cs.direction[0] *= -1;
      cs.direction[1] *= -1;
    }
    cs.velocity = mcu::radsTometers(fabs(cs.dr_vel), wheel_data->radius);
  }

  auto& bls = base_data_.last_state;
  auto& bcs = base_data_.curr_state;
  bcs.acc_direction[0] = bls.acc_direction[0];
  bcs.acc_direction[1] = bls.acc_direction[1];
  bcs.lin_acc = bls.lin_acc;
  bcs.ang_acc = bls.ang_acc;

  if(!base_kinematics_.forwardKinematics(base_data_)){
    return false;
  }

  double icr_nearest_length = 100;
  // const double icr_length = std::hypot(bcs.icr_pos[0], bcs.icr_pos[1]);
  for(const auto& wheel_data : base_data_.wheel_data_vector)
  {
    icr_nearest_length = std::min(std::hypot(wheel_data->pos_on_vehicle[0] - bcs.icr_pos[0],
      wheel_data->pos_on_vehicle[1] - bcs.icr_pos[1]), icr_nearest_length);
  }
  double trans_factor = std::min(10.0, icr_nearest_length) / 10;
  trans_factor = trans_factor * trans_factor / 2;

  if(!steering_mode_ && (base_data_.is_halted || icr_nearest_length > 0.01))
  {
    geometry_msgs::msg::TransformStamped map_trans_odom_msgs;
    if(mcu::getTf(tf_buffer_, "map", "odom", map_trans_odom_msgs, period * 0.01))
    {
      mcu::msgToTf(map_trans_odom_msgs.transform, map_trans_odom_);
      if(map_trans_odom_delay_)
      {
        auto& t1 = map_trans_odom_delay_->getOrigin();
        auto q1 = map_trans_odom_delay_->getRotation();
        const auto& t2 = map_trans_odom_.getOrigin();
        const auto q2 = map_trans_odom_.getRotation();
        const double t_length = (t2 - t1).length();
        const double q_angle = q1.angleShortestPath(q2);
        if(t_length > 0.001 * trans_factor){
          t1 += (t2 - t1) * 0.001 * trans_factor / t_length;
        }else{
          t1 = t2;
        }
        if(q_angle > 0.002 * trans_factor){
          q1 = q1.slerp(q2, 0.002 * trans_factor / q_angle);
        }else{
          q1 = q2;
        }
        map_trans_odom_delay_->setRotation(q1);
      }
      else
      {
        map_trans_odom_delay_ = std::make_unique<tf2::Transform>();
        mcu::msgToTf(map_trans_odom_msgs.transform, *map_trans_odom_delay_);
      }
    }else{
      RCLCPP_WARN(get_node()->get_logger(), "TF buffer lookup failed");
    }
  }

  nav_msgs::msg::Odometry odom_msg;
  geometry_msgs::msg::TransformStamped odom_t_base_msg;

  if(!mcu::updateOdometer(time, period, base_data_.prefix, 
    base_data_.is_halted, base_data_.curr_state, odom_t_base_msg, odom_msg)){
    RCLCPP_WARN(get_node()->get_logger(), "Odom update failed");
  }

  odom_broadcaster_->sendTransform(odom_t_base_msg);
  odom_pub_->publish(odom_msg);

  const tf2::Transform map_trans_odom_delay = 
    (map_trans_odom_delay_) ? *map_trans_odom_delay_ : tf2::Transform::getIdentity();
  const tf2::Transform& odom_t_base = [&]{
    tf2::Transform t;
    mcu::msgToTf(odom_t_base_msg, t);
    return t;
  }();
  map_trans_base_ = map_trans_odom_delay * odom_t_base;
  mcu::baseStateToJointState(bcs, map_trans_base_, state_current_);

  return true;
}

bool MMTrajectoryController::updateBaseCmd(const rclcpp::Duration& period)
{
  bool have_to_stop = false;
  bool want_to_stop = true;
  double period_seconds = period.seconds();

  if(fabs(base_data_.goal_state.lin_vel) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT &&
     fabs(base_data_.goal_state.ang_vel) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT){
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    return false;
  }

  // if(!base_kinematics_.checkSingularity(base_data_, true)){
  //   RCLCPP_WARN(get_node()->get_logger(), "Base target close to the singularity!");
  // }
  if(!base_kinematics_.inverseKinematics(base_data_, true)){
    return false;
  }

  for(auto & wheel_data : base_data_.wheel_data_vector)
  {
    if(!base_kinematics_.wheeledIK(wheel_data, true)){
      return false;
    }
    have_to_stop = have_to_stop || wheel_data->have_to_stop;
    want_to_stop = want_to_stop && wheel_data->want_to_stop;
  }

  steering_mode_ = false;
  if(base_data_.is_halted)
  {
    double dis;
    for(auto & wheel_data : base_data_.wheel_data_vector)
    {
      const double st_a_diff = wheel_data->swerve_joint_limits.acc * period_seconds;
      dis = wheel_data->goal_state.st_pos - wheel_data->last_state.st_pos;
      if((fabs(dis) > st_a_diff) || fabs(wheel_data->last_state.st_vel) > st_a_diff)
      {
        steering_mode_ = true;
        break;
      }
    }
    if(steering_mode_)
    {
      double t_max = 0;
      for(auto & wheel_data : base_data_.wheel_data_vector)
      {
        const double slope = wheel_data->swerve_joint_limits.acc;
        const double v_max = wheel_data->swerve_joint_limits.vel;
        const double pos_diff = wheel_data->goal_state.st_pos - wheel_data->last_state.st_pos;
        const double v0 = wheel_data->last_state.st_vel;
        const double t = mcu::calcMovingTime(pos_diff, v_max, slope, v0, 0.0) * 1.2;
        if (t > t_max){
          t_max = t;
        }
      }

      for(auto & wheel_data : base_data_.wheel_data_vector)
      {
        auto& ls = wheel_data->last_state;
        auto& ns = wheel_data->next_state;
        auto& gs = wheel_data->goal_state;
        if(t_max > period_seconds)
        {
          Eigen::Vector3d res = mcu::calcMinimumJerkTra(ls.st_pos, ls.st_vel, ls.st_acc, 
                                        gs.st_pos, 0.0, 0.0, period_seconds, t_max);
          ns.st_pos = res.coeff(0);
          ns.st_vel = res.coeff(1);
          ns.st_acc = res.coeff(2);
        }
        else
        {
          ns.st_pos = gs.st_pos;
          ns.st_vel = 0;
          ns.st_acc = 0;
        }
        ns.velocity = 0;
        ns.dr_vel = 0;
        ns.dr_acc = 0;
      }
      base_data_.next_state.direction[0] = base_data_.curr_state.direction[0];
      base_data_.next_state.direction[1] = base_data_.curr_state.direction[1];
      base_data_.next_state.acc_direction[0] = base_data_.curr_state.acc_direction[0];
      base_data_.next_state.acc_direction[1] = base_data_.curr_state.acc_direction[1];
      base_data_.next_state.lin_vel = 0;
      base_data_.next_state.ang_vel = 0;
      base_data_.next_state.lin_acc = 0;
      base_data_.next_state.ang_acc = 0;
      return true;
    }
  }
  else if((have_to_stop || want_to_stop))
  {
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    return false;
  }

  // base_data_.next_state.lin_acc = base_data_.dir_acc_max;
  // base_data_.next_state.ang_acc = base_data_.ang_acc_max;
  // mcu::ensureBaseCmdLimit(period, base_data_, base_data_.next_state.lin_acc, 
  //   base_data_.next_state.ang_acc, base_data_.next_state.direction, 
  //   base_data_.next_state.lin_vel, base_data_.next_state.ang_vel);

  base_data_.next_state.direction[0] = base_data_.goal_state.direction[0];
  base_data_.next_state.direction[1] = base_data_.goal_state.direction[1];
  base_data_.next_state.acc_direction[0] = base_data_.goal_state.acc_direction[0];
  base_data_.next_state.acc_direction[1] = base_data_.goal_state.acc_direction[1];
  base_data_.next_state.lin_vel = base_data_.goal_state.lin_vel;
  base_data_.next_state.ang_vel = base_data_.goal_state.ang_vel;
  base_data_.next_state.lin_acc = base_data_.goal_state.lin_acc;
  base_data_.next_state.ang_acc = base_data_.goal_state.ang_acc;



  // if(!base_kinematics_.checkSingularity(base_data_, false)){
  //   RCLCPP_WARN(get_node()->get_logger(), "Base command close to the singularity!");
  // }

  if(!base_kinematics_.inverseKinematics(base_data_, false)){
    return false;
  }
  have_to_stop = false;
  want_to_stop = true;
  for(auto & [wheel_name, wheel_data] : base_data_.wheel_data)
  {
    if(!base_kinematics_.wheeledIK(wheel_data, false)){
      return false;
    }
    have_to_stop = have_to_stop || wheel_data->have_to_stop;
    want_to_stop = want_to_stop && wheel_data->want_to_stop;
  }
  if((have_to_stop || want_to_stop))
  {
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    return false;
  }
  mcu::ensureWheelCmdLimit(period, base_data_);

  // cmd FK here is to update the next state of base data affected by wheel limit
  for(auto& wheel_data : base_data_.wheel_data_vector)
  {
    wheel_data->curr_state.velocity = mcu::radsTometers(
      fabs(wheel_data->next_state.dr_vel), wheel_data->radius);
    wheel_data->curr_state.direction[0] = cos(wheel_data->next_state.st_pos) * 
      std::copysign(1.0, wheel_data->next_state.dr_vel);
    wheel_data->curr_state.direction[1] = sin(wheel_data->next_state.st_pos) * 
      std::copysign(1.0, wheel_data->next_state.dr_vel);
  }
  if(!base_kinematics_.cmdForwardKinematics(base_data_))
  {
    return false;
  }
  return true;
}
}  // namespace mm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mm_controllers::MMTrajectoryController, controller_interface::ControllerInterface)
