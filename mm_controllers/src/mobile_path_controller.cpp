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

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <angles/angles.h>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <controller_interface/helpers.hpp>
#include "mm_controllers/trajectory.hpp"
#include "mm_controllers/mm_controller_utils.hpp"
#include "mm_controllers/mobile_path_controller.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace mcu = mm_controllers::utils;

namespace mm_controllers
{

MobilePathController::MobilePathController()
: controller_interface::ControllerInterface()
{
}

MobilePathController::~MobilePathController()
{
}

controller_interface::CallbackReturn MobilePathController::on_init()
{
  try
  {
    RCLCPP_INFO(get_node()->get_logger(), "Creating controller server");
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    wheel_names_ = params_.wheel_names;
    base_kinematics_ = FourWheelSteeringKinematics();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MobilePathController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    conf.names.reserve(wheel_names_.size() * base_data_.swerve_command_interfaces.size() + 
                       wheel_names_.size() * base_data_.wheel_command_interfaces.size());
   
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
MobilePathController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    conf.names.reserve(wheel_names_.size() * base_data_.swerve_state_interfaces.size() + 
                       wheel_names_.size() * base_data_.wheel_state_interfaces.size());
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

controller_interface::CallbackReturn
MobilePathController::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = get_node();
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Configuring controller interface");

  if (!param_listener_)
  {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  map_trans_odom_ = tf2::Transform();
  map_trans_odom_.setIdentity();
  mcu::initBaseData(params_, base_data_);
  base_kinematics_.configuration(base_data_);
  for(const auto& wheel_name : wheel_names_)
  {
    mcdd::HWInterfaces interfaces;
    interfaces.command.resize(allowed_interface_types_.size());
    interfaces.state.resize(allowed_interface_types_.size());
    wheel_interfaces_.insert(
      std::pair<std::string, mcdd::HWInterfaces>(wheel_name, interfaces));
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

  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());
    
  // action server configuration
  if (params_.allow_partial_joints_goal)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  RCLCPP_INFO(logger,
    "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  // Create the action server that we implement with our followPath method
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowPath>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_path",
    std::bind(&MobilePathController::goal_received_callback, this, _1, _2),
    std::bind(&MobilePathController::goal_cancelled_callback, this, _1),
    std::bind(&MobilePathController::goal_accepted_callback, this, _1));

  odom_broadcaster_ = std::make_unique<
    tf2_ros::TransformBroadcaster>(*this->get_node());
  odom_pub_ = this->get_node()->create_publisher<
    nav_msgs::msg::Odometry>(mcdd::DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobilePathController::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  const auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Activating");

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
          logger, "Expected '%s' command interfaces, got %zu.",
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
          logger, "Expected '%s' state interfaces, got %zu.",
          interface.c_str(), wheel_interfaces_[wheel_name].state[index].size());
        return controller_interface::CallbackReturn::ERROR;
      }
    }
  }

  // (andy)I can't remember why this is here
  if(!tf_buffer_->canTransform(
      base_data_.prefix + "base_footprint", "odom", get_node()->now()))
  {
    tf2::Quaternion q;
    auto const& bs = base_data_.curr_state;
    geometry_msgs::msg::TransformStamped odom_t_base_msg;
    q.setRPY(0, 0, bs.rotation);
    odom_t_base_msg.header.stamp = rclcpp::Time(0);
    odom_t_base_msg.header.frame_id = "odom";
    odom_t_base_msg.child_frame_id = base_data_.prefix + "base_footprint";
    mcu::tfToMsg(tf2::Vector3(bs.position[0], bs.position[1], 0.0), 
                 odom_t_base_msg.transform.translation);
    mcu::tfToMsg(q, odom_t_base_msg.transform.rotation);

    odom_broadcaster_->sendTransform(odom_t_base_msg);
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_point_active_ptr_ = &traj_external_point_ptr_;
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobilePathController::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating");

  // TODO(andyc): Stop the robot and clear interfaces

  for(size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    for(auto& [wheel_name, interfaces] : wheel_interfaces_)
    {
      interfaces.command[index].clear();
      interfaces.state[index].clear();
    }
  }

  release_interfaces();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobilePathController::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up");

  traj_point_active_ptr_ = nullptr;
  if(traj_external_point_ptr_ != nullptr){
    traj_external_point_ptr_.reset();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobilePathController::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Shutting down");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MobilePathController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
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
    return controller_interface::return_type::ERROR;
  }

  if(!steering_mode_ && 
    (base_data_.is_halted || (fabs(bcs.lin_vel) / (fabs(bcs.ang_vel) + 0.001) > 1.5))) 
  {
    geometry_msgs::msg::TransformStamped map_trans_odom_msgs;
    if(mcu::getTf(tf_buffer_, "map", "odom", map_trans_odom_msgs, period * 0.1))
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
        if(t_length > 0.001){
          t1 += (t2 - t1) * 0.001 / t_length;
        }else{
          t1 = t2;
        }
        if(q_angle > 0.01){
          q1 = q1.slerp(q2, 0.01 / q_angle);
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

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg){
    traj_external_point_ptr_->update(*new_external_msg);
    running_point_iter_ = traj_external_point_ptr_->begin();
  }
  if(!traj_point_active_ptr_ || 
    (*traj_point_active_ptr_)->is_executed_already() || 
    !(*traj_point_active_ptr_)->has_trajectory_msg() || 
    !((*traj_point_active_ptr_)->number_of_points() > 0))
  {
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    return controller_interface::return_type::OK;
  }
  try {
    tf2::Transform odom_trans_base, map_trans_base;
    mcu::msgToTf(odom_t_base_msg, odom_trans_base);
    map_trans_base = (map_trans_odom_delay_) ? 
      *map_trans_odom_delay_ * odom_trans_base : odom_trans_base;

    // auto& rp = running_point_iter_->positions;
    auto dist_to_running_point = 
      [&map_trans_base](std::vector<double> pos, double& ang_dist){
        double dist = std::hypot(
          pos.at(0) - map_trans_base.getOrigin()[0],
          pos.at(1) - map_trans_base.getOrigin()[1]);
        ang_dist = std::fabs(pos.at(2) - mcu::quatToAngle(map_trans_base.getRotation()));
        if(ang_dist > 2 * M_PI){
          ang_dist -= 2 * M_PI * int(ang_dist / 2 * M_PI);
        }
        if(ang_dist > M_PI){
          ang_dist = 2 * M_PI - ang_dist;
        }
        return dist;
      };
    
    auto traj_back = traj_external_point_ptr_->end() - 1;
    if(running_point_iter_ < traj_back)
    {
      double ang_dist, pos_dist = dist_to_running_point(
        running_point_iter_->positions, ang_dist);
      double ang_dist_next, pos_dist_next = 
        dist_to_running_point((running_point_iter_ + 1)->positions, ang_dist_next);
      if((pos_dist < mcdd::DEFAULT_TOLERANCE * 50 && 
          ang_dist < mcdd::DEFAULT_TOLERANCE * 100) ||
         (ang_dist_next < ang_dist && pos_dist_next < pos_dist))
      {
        running_point_iter_ += 1;
      }
    }

    this->computeVelocityCommands(
      time, period, map_trans_base, base_data_, base_data_.goal_state);

    if(updateBaseCmd(period))
    {
      RCLCPP_DEBUG(get_node()->get_logger(), 
        "Publishing velocity at time %.2f", get_node()->now().seconds());
      mcu::setBaseCmd(wheel_interfaces_, base_data_);
    }

    const auto active_goal = *rt_active_goal_.readFromRT();
    if (active_goal)
    {
      std::shared_ptr<FollowPath::Feedback> feedback = 
        std::make_shared<FollowPath::Feedback>();
      feedback->speed = base_data_.next_state.lin_vel;
      feedback->distance_to_goal = 0;
      active_goal->setFeedback(feedback);
    
      if (running_point_iter_ == traj_back)
      {
        // update goal state using goal pose on odom frame
        tf2::Quaternion q;
        tf2::Transform map_trans_goal, odom_trans_goal;
        const auto& p = traj_back->positions;
        q.setRPY(0.0, 0.0, p.at(2));
        map_trans_goal.setRotation(q);
        map_trans_goal.setOrigin(tf2::Vector3(p.at(0), p.at(1), 0.0));
        odom_trans_goal = map_trans_odom_.inverse() * map_trans_goal;
        base_data_.goal_state.position[0] = odom_trans_goal.getOrigin().getX();
        base_data_.goal_state.position[1] = odom_trans_goal.getOrigin().getY();
        base_data_.goal_state.rotation = mcu::quatToAngle(odom_trans_goal.getRotation());

        if(isGoalReached(base_data_))
        {
          RCLCPP_INFO(get_node()->get_logger(), "1. Reached the goal!");
          auto res = std::make_shared<FollowPath::Result>();
          active_goal->setSucceeded(res);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          (*traj_point_active_ptr_)->set_executed();
          RCLCPP_INFO(get_node()->get_logger(), "Reached the goal!");
        }
      }
    }

    return controller_interface::return_type::OK;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), e.what());
    mcu::baseHalt(period, wheel_interfaces_, base_data_);
    const auto active_goal = *rt_active_goal_.readFromRT();
    if(active_goal)
    {
      auto result = std::make_shared<FollowPath::Result>();
      active_goal->setAborted(result);
      // TODO(matthew-reynolds): Need a lock-free write here
      // See https://github.com/ros-controls/ros2_controllers/issues/168
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void MobilePathController::setPlannerPath(const nav_msgs::msg::Path & path_msg)
{
  std::shared_ptr<std::list<Eigen::VectorXd>> points = 
    mcu::convertPathToPoints(path_msg);

  auto traj_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj_msg->header = path_msg.header;
  traj_msg->joint_names.push_back("base_x");
  traj_msg->joint_names.push_back("base_y");
  traj_msg->joint_names.push_back("base_rz");

  if(points->size() == 1){
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(points->front()[0]);
    point.positions.push_back(points->front()[1]);
    point.positions.push_back(points->front()[2]);
    point.time_from_start = rclcpp::Duration(0, 0);
    traj_msg->points.push_back(point);
    return;
  }

  double sample_time = 0.1;
  Eigen::Vector3d max_vel, max_acc;
  max_vel << base_data_.dir_vel_max / 2, 
    base_data_.dir_vel_max / 2, base_data_.ang_vel_max / 2;
  max_acc << base_data_.dir_acc_max / 2, 
    base_data_.dir_acc_max / 2, base_data_.ang_acc_max / 2;

  TrajectoryParameterization parameterized(
    Path(*points, sample_time), max_vel, max_acc, DEFAULT_TIMESTEP);

  if (!parameterized.isValid())
  {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Unable to parameterize trajectory.");
    return;
  }

  traj_msg->points = *(mcu::parameterizedToMsg(parameterized, sample_time));
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

rclcpp_action::GoalResponse MobilePathController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if(goal->path.poses.empty()){
    RCLCPP_ERROR(
    get_node()->get_logger(), "Can't accept new action goals. path is empty.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MobilePathController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_DEBUG(
      get_node()->get_logger(), 
      "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    auto action_res = std::make_shared<FollowPath::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MobilePathController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowPath>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Passing new path to controller.");
  auto& goal = goal_handle->get_goal();
  preempt_active_goal();
  setPlannerPath(goal->path);

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void MobilePathController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<FollowPath::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

bool MobilePathController::isGoalReached(const mcdd::BaseData& base_data)
{
  const auto& cs = base_data.curr_state;
  const auto& ns = base_data.next_state;
  const auto& gs = base_data.goal_state;
  if(fabs(ns.lin_vel) > mcdd::DEFAULT_HALTED_THRESHOLD * 10 || 
     fabs(ns.ang_vel) > mcdd::DEFAULT_HALTED_THRESHOLD * 10){
    return false;
  }
  double pos_dis = std::hypot(gs.position[0] - cs.position[0],
                              gs.position[1] - cs.position[1]);
  double ang_dis = fabs(gs.rotation - cs.rotation);
  if(pos_dis > mcdd::DEFAULT_TOLERANCE * 10 || 
     ang_dis > mcdd::DEFAULT_TOLERANCE * 10){
    return false;
  }
  return true;
}

bool MobilePathController::updateBaseCmd(const rclcpp::Duration& period)
{
  bool have_to_stop = false;
  bool want_to_stop = true;
  double period_seconds = period.seconds();

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
  // mcu::ensureBaseCmdLimit(
  //   period, base_data_, base_data_.next_state.lin_acc, 
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
    wheel_data->curr_state.velocity = 
      mcu::radsTometers(fabs(wheel_data->next_state.dr_vel), wheel_data->radius);
    wheel_data->curr_state.direction[0] = 
      cos(wheel_data->next_state.st_pos) * std::copysign(1.0, wheel_data->next_state.dr_vel);
    wheel_data->curr_state.direction[1] = 
      sin(wheel_data->next_state.st_pos) * std::copysign(1.0, wheel_data->next_state.dr_vel);
  }
  if(!base_kinematics_.cmdForwardKinematics(base_data_))
  {
    return false;
  }
  return true;
}

void MobilePathController::computeVelocityCommands(
  const rclcpp::Time& time, const rclcpp::Duration& period, const tf2::Transform& cur_pose,
  const mcdd::BaseData& base_data, mcdd::BaseState& goal_state)
{
  // currently carrying out a trajectory
  auto & bl = base_data.last_state;
  mcdd::Double2 v_dir, a_dir;
  double curr_ang = mcu::quatToAngle(cur_pose.getRotation());
  double sin_theta = sin(curr_ang);
  double cos_theta = cos(curr_ang);
  v_dir[0] = cos_theta * bl.direction[0] - sin_theta * bl.direction[1];
  v_dir[1] = cos_theta * bl.direction[1] + sin_theta * bl.direction[0];
  a_dir[0] = cos_theta * bl.acc_direction[0] - sin_theta * bl.acc_direction[1];
  a_dir[1] = cos_theta * bl.acc_direction[1] + sin_theta * bl.acc_direction[0];

  trajectory_msgs::msg::JointTrajectoryPoint state_desired, state_current;
  state_current.positions.reserve(3);
  state_current.velocities.reserve(3);
  state_current.accelerations.reserve(3);
  state_desired.positions.resize(3, 0.0);
  state_desired.velocities.resize(3, 0.0);
  state_desired.accelerations.resize(3, 0.0);
  state_current.positions.push_back(cur_pose.getOrigin().getX());
  state_current.positions.push_back(cur_pose.getOrigin().getY());
  state_current.positions.push_back(curr_ang);
  state_current.velocities.push_back(v_dir[0] * bl.lin_vel);
  state_current.velocities.push_back(v_dir[1] * bl.lin_vel);
  state_current.velocities.push_back(bl.ang_vel);
  state_current.accelerations.push_back(a_dir[0] * bl.lin_acc);
  state_current.accelerations.push_back(a_dir[1] * bl.lin_acc);
  state_current.accelerations.push_back(bl.ang_acc);

  // if sampling the first time, set the point before you sample
  if (!(*traj_point_active_ptr_)->is_sampled_already())
  {
    (*traj_point_active_ptr_)->set_point_before_trajectory_msg(time, state_current);
  }

  // find segment for current timestamp
  TrajectoryPointConstIter start_segment_itr, end_segment_itr;
  const bool valid_point =
    (*traj_point_active_ptr_)
      ->sample(time, interpolation_method_, state_desired, start_segment_itr, end_segment_itr);
  if(valid_point)
  {
    if(end_segment_itr > running_point_iter_)
    {
      auto& scp = state_current.positions;
      const auto& scv = state_current.velocities;
      const auto& sgp = running_point_iter_->positions;
      const auto& sgv = running_point_iter_->velocities;
      const double tmp_curr_rz = scp[2];
      scp[2] = sgp[2] - angles::shortest_angular_distance(scp[2], sgp[2]);
      RCLCPP_INFO(get_node()->get_logger(),
        "Behind the planned trajectory! %d", 
        int(running_point_iter_ - (*traj_point_active_ptr_)->begin()));
      const double t_x = mcu::calcMovingTime(sgp[0] - scp[0], base_data.dir_vel_max, 
                                             base_data.dir_acc_max, scv[0], sgv[0]);
      const double t_y = mcu::calcMovingTime(sgp[1] - scp[1], base_data.dir_vel_max,
                                             base_data.dir_acc_max, scv[1], sgv[1]);
      const double t_z = mcu::calcMovingTime(sgp[2] - scp[2], base_data.ang_vel_max, 
                                             base_data.ang_acc_max, scv[2], sgv[2]);
      // 1.2 here because time calc doesn't consider jerk,                                                        
      // so the actual running time is longer
      double t_max = std::max(std::max(t_x, t_y), t_z) * 1.2;
      rclcpp::Duration tt = rclcpp::Duration(int32_t(t_max), uint32_t((t_max*1e9)) % uint32_t(1e9));
      if(tt < period){
        state_desired = *running_point_iter_;
      }else{
        (*traj_point_active_ptr_)->interpolate_between_points(time, state_current, 
          time + tt, *running_point_iter_, time + period, state_desired);
        state_current.positions[2] = tmp_curr_rz;
      }
    }

    double cmd_vx = cos_theta * state_desired.velocities[0] + 
      sin_theta * state_desired.velocities[1];
    double cmd_vy = cos_theta * state_desired.velocities[1] - 
      sin_theta * state_desired.velocities[0];
    double cmd_ax = cos_theta * state_desired.accelerations[0] + 
      sin_theta * state_desired.accelerations[1];
    double cmd_ay = cos_theta * state_desired.accelerations[1] - 
      sin_theta * state_desired.accelerations[0];
    goal_state.lin_vel = std::hypot(cmd_vx, cmd_vy);
    goal_state.ang_vel = state_desired.velocities[2];
    goal_state.lin_acc = std::hypot(cmd_ax, cmd_ay);
    goal_state.ang_acc = state_desired.accelerations[2];
    if(goal_state.lin_vel > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
    {
      goal_state.direction[0] = cmd_vx / goal_state.lin_vel;
      goal_state.direction[1] = cmd_vy / goal_state.lin_vel;
    }
    if(goal_state.lin_acc > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
    {
      goal_state.acc_direction[0] = cmd_ax / goal_state.lin_acc;
      goal_state.acc_direction[1] = cmd_ay / goal_state.lin_acc;
    }
    return;
  }
}
}  // namespace mm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mm_controllers::MobilePathController, controller_interface::ControllerInterface)