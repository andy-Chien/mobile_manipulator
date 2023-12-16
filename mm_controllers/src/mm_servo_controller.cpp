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

#include <string>
#include <thread>
#include <future>
#include <Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "mm_controllers/mm_servo_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace mm_controllers
{
MMServoController::MMServoController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MMServoController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    subscriber_is_active_ = false;
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    rt_mm_ik_res_ptr_ = realtime_tools::RealtimeBuffer<
      std::shared_ptr<mm_msgs::srv::IK_Response>>(nullptr);
    last_mm_ik_res_ = std::shared_ptr<mm_msgs::srv::IK::Response>();
    base_kinematics_ = FourWheelSteeringKinematics();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMServoController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    mcu::initBaseData(params_, base_data_);
    for(const auto& arm_name : params_.arm_names)
    {
      mcdd::ArmData arm_data;
      arm_data.arm_name = arm_name;
      mcu::initArmData(params_, arm_data);
      arm_data_.insert(std::pair<std::string, mcdd::ArmData>(arm_name, arm_data));
    }

    base_kinematics_.configuration(base_data_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    mm_ik_client_ = this->get_node()->create_client<mm_msgs::srv::IK>("/IK_service");
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this->get_node());
    odom_pub_ = this->get_node()->create_publisher<
      nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    cmd_sub_  = this->get_node()->create_subscription<
      CmdType>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<CmdType> msg) -> void {
        if(!subscriber_is_active_)
        {
          RCLCPP_WARN(
            get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        rt_command_ptr_.writeFromNonRT(msg);
      });

    init_robot_state_thread_future_ = std::async(std::launch::async,
      mcu::initRobotState, param_listener_, std::ref(robot_state_));
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMServoController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for(const auto & arm_name : params_.arm_names)
  {
    std::vector<std::vector<std::reference_wrapper<
      hardware_interface::LoanedCommandInterface>>> arm_ordered_multi_command_interfaces;
    std::vector<std::vector<std::reference_wrapper<
      hardware_interface::LoanedStateInterface>>> arm_ordered_multi_state_interfaces;
    for(const auto & interface : arm_data_[arm_name].command_interface_types)
    {
      std::vector<std::reference_wrapper<
        hardware_interface::LoanedCommandInterface>> ordered_interfaces;
      if(!controller_interface::get_ordered_interfaces(
        command_interfaces_, arm_data_[arm_name].joint_names,
        interface, ordered_interfaces))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' command interfaces, got %zu.",
          interface.c_str(), ordered_interfaces.size());
        return controller_interface::CallbackReturn::ERROR;
      }
      arm_ordered_multi_command_interfaces.push_back(ordered_interfaces);
    }
    for(const auto & interface : arm_data_[arm_name].state_interface_types)
    {
      std::vector<std::reference_wrapper<
        hardware_interface::LoanedStateInterface>> ordered_interfaces;
      if(!controller_interface::get_ordered_interfaces(
        state_interfaces_, arm_data_[arm_name].joint_names, 
        interface, ordered_interfaces))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' state interfaces, got %zu.",
          interface.c_str(), ordered_interfaces.size());
        return controller_interface::CallbackReturn::ERROR;
      }
      arm_ordered_multi_state_interfaces.push_back(ordered_interfaces);
    }
    arm_interfaces_.insert(std::pair<std::string, mcdd::HWInterfaces>(arm_name, 
      mcdd::HWInterfaces{
        arm_ordered_multi_command_interfaces,
        arm_ordered_multi_state_interfaces}));
  }

  for(const auto & wheel_name : params_.wheel_names)
  {
    std::vector<std::vector<std::reference_wrapper<
      hardware_interface::LoanedCommandInterface>>> wheel_ordered_multi_command_interfaces;
    std::vector<std::vector<std::reference_wrapper<
      hardware_interface::LoanedStateInterface>>> wheel_ordered_multi_state_interfaces;
    std::vector<std::string> swerve_joint_names{
      base_data_.wheel_data.at(wheel_name)->joints_name[0]};
    std::vector<std::string> wheel_joint_names{
      base_data_.wheel_data.at(wheel_name)->joints_name[1]};
    std::string swerve_joint_name = 
      base_data_.wheel_data.at(wheel_name)->joints_name[0];
    std::string wheel_joint_name = 
      base_data_.wheel_data.at(wheel_name)->joints_name[1];
    for(const auto & interface : allowed_interface_types_)
    {
      std::vector<std::reference_wrapper<
        hardware_interface::LoanedCommandInterface>> ordered_interfaces;
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
      if(!controller_interface::get_ordered_interfaces(
        command_interfaces_, joint_names, interface, ordered_interfaces))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' command interfaces, got %zu.",
          interface.c_str(), ordered_interfaces.size());
        return controller_interface::CallbackReturn::ERROR;
      }
      wheel_ordered_multi_command_interfaces.push_back(ordered_interfaces);
    }
    for(const auto & interface : allowed_interface_types_)
    {
      std::vector<std::reference_wrapper<
        hardware_interface::LoanedStateInterface>> ordered_interfaces;
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
      if(!controller_interface::get_ordered_interfaces(
        state_interfaces_, joint_names, interface, ordered_interfaces))
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Expected '%s' state interfaces, got %zu.",
          interface.c_str(), ordered_interfaces.size());
        return controller_interface::CallbackReturn::ERROR;
      }
      wheel_ordered_multi_state_interfaces.push_back(ordered_interfaces);
    }

    wheel_interfaces_.insert(std::pair<std::string, mcdd::HWInterfaces>(wheel_name, 
      mcdd::HWInterfaces{
        wheel_ordered_multi_command_interfaces,
        wheel_ordered_multi_state_interfaces}));
  }

  geometry_msgs::msg::TransformStamped odom_trans_base_msgs;
  if(mcu::getTf(tf_buffer_, "odom", "mobile_base_footprint", \
    odom_trans_base_msgs, rclcpp::Duration(int32_t(0), uint32_t(5e8))))
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

  for(auto& [arm_name, arm_data] : arm_data_)
  {
    arm_data.curr_state.clear();
    for(auto const & pos_interface : arm_interfaces_[arm_name].state[0]){
      arm_data.curr_state.jnt_pos.push_back(
        pos_interface.get().get_value());
    }
    for(auto const & vel_interface : arm_interfaces_[arm_name].state[1]){
      arm_data.curr_state.jnt_vel.push_back(
        vel_interface.get().get_value());
      arm_data.curr_state.jnt_acc.push_back(0);
    }
    arm_data.curr_state.pose = std::vector<double>(
      7, std::numeric_limits<double>::infinity());
    
    arm_data.last_state = arm_data.curr_state;
    arm_data.next_state = arm_data.curr_state;
    arm_data.goal_state = arm_data.curr_state;

    if(arm_data.curr_state.jnt_pos.size() != arm_data.joint_names.size()){
      RCLCPP_ERROR(
        get_node()->get_logger(), "Size of joint pos state is not same as joint names!");
      return controller_interface::CallbackReturn::ERROR;
    }
    arm_is_halted_.insert(std::pair<std::string, bool>(arm_name, false));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
  MMServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = 
    controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    int arm_interface_size = 0;
    for(const auto& arm_name : params_.arm_names)
    {
      arm_interface_size += 
        arm_data_.at(arm_name).joint_names.size() * 
        arm_data_.at(arm_name).command_interface_types.size();
    }
    command_interfaces_config.names.reserve(
      arm_interface_size +
      params_.wheel_names.size() * base_data_.swerve_command_interfaces.size() + 
      params_.wheel_names.size() * base_data_.wheel_command_interfaces.size());
    for(const auto& arm_name : params_.arm_names)
    {
      for(const auto & joint_name : arm_data_.at(arm_name).joint_names)
      {
        for(const auto & interface_type : arm_data_.at(arm_name).command_interface_types)
        {
          command_interfaces_config.names.push_back(joint_name + "/" + interface_type);
        }
      }
    }
    for(const auto & wheel_name : params_.wheel_names)
    {
      for(const auto & interface_type : base_data_.swerve_command_interfaces)
      {
        command_interfaces_config.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[0] + "/" + interface_type);
      }
      for(const auto & interface_type : base_data_.wheel_command_interfaces)
      {
        command_interfaces_config.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[1] + "/" + interface_type);
      }
    }
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, 
      "Exception thrown during command_interface_configuration with message: %s \n", e.what());
    return command_interfaces_config;
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration 
  MMServoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    int arm_interface_size = 0;
    for(const auto& arm_name : params_.arm_names)
    {
      arm_interface_size +=
        arm_data_.at(arm_name).joint_names.size() * 
        arm_data_.at(arm_name).state_interface_types.size();
    }
    state_interfaces_config.names.reserve(
      arm_interface_size +
      params_.wheel_names.size() * base_data_.swerve_state_interfaces.size() + 
      params_.wheel_names.size() * base_data_.wheel_state_interfaces.size());
    for(const auto& arm_name : params_.arm_names)
    {
      for(const auto & joint_name : arm_data_.at(arm_name).joint_names)
      {
        for(const auto & interface_type : arm_data_.at(arm_name).state_interface_types)
        {
          state_interfaces_config.names.push_back(joint_name + "/" + interface_type);
        }
      }
    }
    for(const auto & wheel_name : params_.wheel_names)
    {
      for(const auto & interface_type : base_data_.swerve_state_interfaces)
      {
        state_interfaces_config.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[0] + "/" + interface_type);
      }
      for(const auto & interface_type : base_data_.wheel_state_interfaces)
      {
        state_interfaces_config.names.push_back(
          base_data_.wheel_data.at(wheel_name)->joints_name[1] + "/" + interface_type);
      }
    }
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, 
      "Exception thrown during state_interface_configuration with message: %s \n", e.what());
    return state_interfaces_config;
  }
  return state_interfaces_config;
}

controller_interface::return_type MMServoController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if(!subscriber_is_active_)
  {
    if(init_robot_state_thread_future_.wait_for(
      std::chrono::nanoseconds(1)) != std::future_status::ready)
    {
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "waiting for robot state init thread.");
      return controller_interface::return_type::OK;
    }
    else{
      subscriber_is_active_ = true;
    }
  }
  bool update_base_cmd = false;
  std::map<std::string, bool> update_arm_cmd;
  auto last_cmd_msg = rt_command_ptr_.readFromRT();
  for(auto const& [wheel_name, wheel_interface] : wheel_interfaces_)
  {
    mcdd::WheelData* wheel_data = base_data_.wheel_data[wheel_name];
    mcdd::WheelState& cs = wheel_data->curr_state;
    mcdd::WheelState& ls = wheel_data->last_state;
    cs.st_pos = wheel_interface.state[0][0].get().get_value();
    cs.dr_vel = wheel_interface.state[1][1].get().get_value();
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

  if(!base_kinematics_.forwardKinematics(base_data_)){
    return controller_interface::return_type::ERROR;
  }

  {
    nav_msgs::msg::Odometry odom_msg;
    geometry_msgs::msg::TransformStamped odom_t_base_msg;

    if(!mcu::updateOdometer(time, period, base_data_.prefix, 
      base_data_.is_halted, base_data_.curr_state, odom_t_base_msg, odom_msg)){
      RCLCPP_WARN(get_node()->get_logger(), "Odom update failed");
    }

    odom_broadcaster_->sendTransform(odom_t_base_msg);
    odom_pub_->publish(odom_msg);
  }

  for(auto const & [arm_name, arm_interface] : arm_interfaces_)
  {
    arm_is_halted_.at(arm_name) = true;
    std::vector<double> positions, velocities;
    for(auto const & pos_interface : arm_interface.state[0])
    {
      positions.push_back(pos_interface.get().get_value());
    }
    for(auto const & vel_interface : arm_interface.state[1])
    {
      double vel = vel_interface.get().get_value();
      velocities.push_back(vel);
      arm_is_halted_.at(arm_name) = 
        arm_is_halted_.at(arm_name) && (fabs(vel) < mcdd::DEFAULT_HALTED_THRESHOLD);
    }
    arm_data_[arm_name].curr_state.jnt_pos = positions;
    arm_data_[arm_name].curr_state.jnt_vel = velocities;
    update_arm_cmd[arm_name] = false;
  }

  if(!last_cmd_msg || !(*last_cmd_msg))
  {
    return controller_interface::return_type::OK;
  }

  if(((*last_cmd_msg)->control_mode == mm_msgs::msg::ServoControl::ARM) ||
    ((*last_cmd_msg)->control_mode == mm_msgs::msg::ServoControl::HYBRID) ||
    ((*last_cmd_msg)->control_mode == mm_msgs::msg::ServoControl::COMBINE))
  {
    for(auto& arm_data : arm_data_)
    {
      if(!mcu::getFKResult(arm_data.second.joint_names, arm_data.second.curr_state.jnt_pos,
        params_.kinematics.arm_ik_base_link, params_.kinematics.arm_ik_end_link,
        robot_state_, arm_data.second.curr_state.pose))
      {
        return controller_interface::return_type::ERROR;
      }
    }
  }

  switch ((*last_cmd_msg)->control_mode)
  {
  case mm_msgs::msg::ServoControl::STOP:
    this->baseHalt(period);
    for(auto const& arm_name : params_.arm_names)
    {
      this->armHalt(arm_data_[arm_name], period);
    }
    return controller_interface::return_type::OK;

  case mm_msgs::msg::ServoControl::ARM:
    this->baseHalt(period);
    for(auto& [arm_name, arm_data] : arm_data_)
    {
      update_arm_cmd.at(arm_name) = updateArmCmd(arm_data, (*last_cmd_msg)->arm_cmd,
        (*last_cmd_msg)->arm_cmd_frame, tf2::Transform::getIdentity(), time, period);
    }
    break;

  case mm_msgs::msg::ServoControl::BASE:
    for(auto const& arm_name : params_.arm_names)
    {
      mcu::armHalt(period, arm_interfaces_[arm_name], arm_data_[arm_name]);
    }
    update_base_cmd = updateBaseCmd(
      (*last_cmd_msg)->base_cmd, (*last_cmd_msg)->base_cmd_frame, period);
    break;

  case mm_msgs::msg::ServoControl::HYBRID:
    if(base_data_.is_halted){
      base_moving_trans_.setIdentity();
    }else{
      mcu::recordBaseMoving(period, base_data_.curr_state, base_moving_trans_);
    }

    update_base_cmd = updateBaseCmd(
      (*last_cmd_msg)->base_cmd, (*last_cmd_msg)->base_cmd_frame, period);
    for(auto& [arm_name, arm_data] : arm_data_)
    {
      update_arm_cmd.at(arm_name) = updateArmCmd(arm_data, (*last_cmd_msg)->arm_cmd,
        (*last_cmd_msg)->arm_cmd_frame, base_moving_trans_.inverse(), time, period);
      if(!update_arm_cmd[arm_name])
      {
        this->armHalt(arm_data, period);
        this->baseHalt(period);
        return controller_interface::return_type::OK;
      }
    }
    base_moving_trans_.setIdentity();
    break;

  case mm_msgs::msg::ServoControl::COMBINE:
    {
      const auto cmd_l = (*last_cmd_msg)->arm_cmd.linear;
      const auto cmd_a = (*last_cmd_msg)->arm_cmd.angular;
      if(std::hypot(cmd_l.x, cmd_l.y, cmd_l.z) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT &&
        std::hypot(cmd_a.x, cmd_a.y, cmd_a.z) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
      {
        this->baseHalt(period);
        for(auto const& arm_name : params_.arm_names)
        {
          this->armHalt(arm_data_[arm_name], period);
        }
        return controller_interface::return_type::OK;
      }
    }
    base_moving_trans_.setIdentity();
    for(auto& [arm_name, arm_data] : arm_data_)
    {
      using vec2 = std::tuple<std::vector<double>, std::vector<double>>;
      const auto& [reference_pose, reference_pos] = (mcu::checkTolerance(
        arm_data.curr_state.pose, arm_data.last_state.pose,
        arm_data.curr_state.jnt_pos, arm_data.last_state.jnt_pos)) ? 
        vec2(arm_data.last_state.pose, arm_data.last_state.jnt_pos) :
        vec2(arm_data.curr_state.pose, arm_data.curr_state.jnt_pos);

      auto last_mm_ik_res = rt_mm_ik_res_ptr_.readFromRT();
      if(*last_mm_ik_res != last_mm_ik_res_ || !(*last_mm_ik_res))
      {
        last_mm_ik_res_ = *last_mm_ik_res;
      
        const auto cmd = (*last_cmd_msg)->arm_cmd;
        const double cmd_sample_time  = rclcpp::Duration(int32_t(0), uint32_t(5e8)).seconds();

        tf2::Quaternion cmd_quat, base_quat_tool, world_quat_base, target_quat;
        tf2::Transform cmd_trans, base_trans_tool, world_trans_base, target_trans;


        base_quat_tool = tf2::Quaternion(
          reference_pose[4], 
          reference_pose[5],
          reference_pose[6], 
          reference_pose[3]
        );
        base_trans_tool.setOrigin(tf2::Vector3(
          reference_pose[0], 
          reference_pose[1], 
          reference_pose[2]
        ));
        base_trans_tool.setRotation(base_quat_tool);

        cmd_quat.setRPY(
          cmd.angular.x * cmd_sample_time,
          cmd.angular.y * cmd_sample_time,
          cmd.angular.z * cmd_sample_time
        );
        cmd_trans.setIdentity();
        cmd_trans.setOrigin(tf2::Vector3(
          cmd.linear.x * cmd_sample_time,
          cmd.linear.y * cmd_sample_time,
          cmd.linear.z * cmd_sample_time
        ));

        // TODO(Andyc): 0.5 here should read from urdf or param system
        world_quat_base.setRPY(0, 0, base_data_.curr_state.rotation);
        world_trans_base.setRotation(world_quat_base);
        world_trans_base.setOrigin(tf2::Vector3(
          base_data_.curr_state.position[0], base_data_.curr_state.position[1], 0.5
        ));// z dis from base footprint to arm base

        switch ((*last_cmd_msg)->arm_cmd_frame)
        {
        case mm_msgs::msg::ServoControl::BASE_FRAME:
          target_quat = world_quat_base * cmd_quat * base_quat_tool;
          target_trans = world_trans_base * cmd_trans * base_trans_tool;
          break;
        case mm_msgs::msg::ServoControl::WORLD_FRAME:
          target_quat = cmd_quat * world_quat_base * base_quat_tool;
          target_trans = cmd_trans * world_trans_base * base_trans_tool;
          break;
        case mm_msgs::msg::ServoControl::TOOL_FRAME:
          target_quat = world_quat_base * base_quat_tool * cmd_quat;
          target_trans = world_trans_base * base_trans_tool * cmd_trans;
          break;
        default:
          RCLCPP_ERROR(get_node()->get_logger(), 
            "Arm command frame %d not support!", (*last_cmd_msg)->arm_cmd_frame);
          return controller_interface::return_type::ERROR;
        }
        target_trans.setRotation(target_quat);

        const tf2::Transform curr_trans = world_trans_base * base_trans_tool;

        // pub goal pose odom
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "goal_footprint";

        odom_trans.transform.translation.x = target_trans.getOrigin()[0];
        odom_trans.transform.translation.y = target_trans.getOrigin()[1];
        odom_trans.transform.translation.z = target_trans.getOrigin()[2];
        odom_trans.transform.rotation.x = target_quat.getX();
        odom_trans.transform.rotation.y = target_quat.getY();
        odom_trans.transform.rotation.z = target_quat.getZ();
        odom_trans.transform.rotation.w = target_quat.getW();
        odom_broadcaster_->sendTransform(odom_trans);
        // end of goal pose pub

        if(!this->getMMIKResult(base_data_.curr_state, arm_data.curr_state, curr_trans, target_trans)){
          RCLCPP_ERROR(get_node()->get_logger(), "getMMIKResult failed");
          return controller_interface::return_type::ERROR;
        }
      }

      if(!last_mm_ik_res || !(*last_mm_ik_res)){
        return controller_interface::return_type::OK;
      }

      geometry_msgs::msg::Twist base_cmd;
      if(last_mm_ik_res_->joint_speed.size() == 9)
      {
        base_cmd.linear.x = last_mm_ik_res_->joint_speed[0];
        base_cmd.linear.y = last_mm_ik_res_->joint_speed[1];
        base_cmd.angular.z = last_mm_ik_res_->joint_speed[2];
        for(unsigned i=0; i < arm_data.goal_state.jnt_vel.size(); i++){
          arm_data.goal_state.jnt_vel[i] = last_mm_ik_res_->joint_speed.at(i+3);
        }
      }else{
        RCLCPP_WARN(get_node()->get_logger(), "Size of mm ik res is wrong.");
        return controller_interface::return_type::ERROR;
      }

      update_base_cmd = updateBaseCmd(
        base_cmd, mm_msgs::msg::ServoControl::WORLD_FRAME, period);

      const double base_vel_factor = std::min(
        fabs((base_data_.next_state.lin_vel - base_data_.last_state.lin_vel) / 
             (base_data_.goal_state.lin_vel - base_data_.last_state.lin_vel)),
        fabs((base_data_.next_state.ang_vel - base_data_.last_state.ang_vel) / 
             (base_data_.goal_state.ang_vel - base_data_.last_state.ang_vel))
      );
      std::cout<<"base_vel_factor = "<<base_vel_factor<<", "<<std::isnan(base_vel_factor)<<std::endl;
      if(!std::isnan(base_vel_factor) && base_vel_factor < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
      {
        this->armHalt(arm_data, period);
        update_arm_cmd.at(arm_name) = false;
      }
      else
      {
        std::cout<<"jnt_vel = ";
        for(size_t i=0; i<arm_data.joint_names.size(); i++)
        {
          arm_data.goal_state.jnt_vel[i] = arm_data.last_state.jnt_vel[i] +
            (arm_data.goal_state.jnt_vel[i] - arm_data.last_state.jnt_vel[i]) * 
            (std::isnan(base_vel_factor) ? 1.0 : base_vel_factor);
          arm_data.goal_state.jnt_pos[i] = 
            reference_pos[i] + arm_data.goal_state.jnt_vel[i] * period.seconds();
          arm_data.goal_state.jnt_acc[i] = 
            (arm_data.goal_state.jnt_vel[i] - arm_data.last_state.jnt_vel[i]) / period.seconds();
          std::cout<<arm_data.goal_state.jnt_vel[i]<<", "<<arm_data.goal_state.jnt_pos[i]<<", ";
        }
        std::cout<<std::endl;
        mcu::ensureArmCmdLimit(period, arm_data);
        update_arm_cmd.at(arm_name) = true;
      }
      // update pose cmd to fit joint position cmds
      while(!mcu::getFKResult(arm_data.joint_names, arm_data.next_state.jnt_pos,
        params_.kinematics.arm_ik_base_link, params_.kinematics.arm_ik_end_link,
        robot_state_, arm_data.next_state.pose))
      {
        RCLCPP_WARN(get_node()->get_logger(), 
          "pose cmd update failed, try again!");
      }
    }
    break;

  case mm_msgs::msg::ServoControl::SINGLE_JOINT:
    this->baseHalt(period);
    for(auto& [arm_name, arm_data] : arm_data_)
    {
      arm_data.goal_state.jnt_pos = arm_data.last_state.jnt_pos;
      const auto itr = std::find(
        arm_data.joint_names.begin(), 
        arm_data.joint_names.end(), (*last_cmd_msg)->joint);
      if(itr != arm_data.joint_names.cend())
      {
        const auto index = std::distance(arm_data.joint_names.begin(), itr);
        arm_data.goal_state.jnt_pos.at(index) +=
          (*last_cmd_msg)->value * period.seconds();
        arm_data.goal_state.jnt_vel.at(index) = 
          (*last_cmd_msg)->value;
        arm_data.goal_state.jnt_acc.at(index) = (arm_data.goal_state.jnt_vel.at(index) - 
          arm_data.last_state.jnt_vel.at(index)) / period.seconds();
        mcu::ensureArmCmdLimit(period, arm_data);
        update_arm_cmd.at(arm_name) = true;
      }
    }
    break;

  default:
    RCLCPP_ERROR(get_node()->get_logger(), 
      "Control mode %d not supported!", (*last_cmd_msg)->control_mode);
    return controller_interface::return_type::ERROR;
  }

  if(update_base_cmd)
  {
    mcu::setBaseCmd(wheel_interfaces_, base_data_);
  }
  for(auto const& [arm_name, update] : update_arm_cmd)
  {
    if(update)
    {
      mcu::setArmCmd(arm_interfaces_[arm_name], arm_data_[arm_name]);
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MMServoController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  subscriber_is_active_ = false;
  arm_interfaces_.clear();
  wheel_interfaces_.clear();
  arm_is_halted_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMServoController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_.arm_names.clear();
  params_.wheel_names.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMServoController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MMServoController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool MMServoController::getMMIKResult(
  const mcdd::BaseState& base_state, const mcdd::ArmState& arm_state, 
  const tf2::Transform& curr_pose, const tf2::Transform& goal_pose)
{
  if(!mm_ik_client_->service_is_ready())
  {
    RCLCPP_WARN(get_node()->get_logger(), "mm IK service not started");
    return false;
  }

  auto ik_req = std::make_shared<mm_msgs::srv::IK::Request>();
  ik_req->goal_pose.reserve(16);
  ik_req->current_pose.reserve(16);
  ik_req->current_joint.reserve(9);
  const tf2::Vector3& curr_pos = curr_pose.getOrigin();
  const tf2::Vector3& goal_pos = goal_pose.getOrigin();
  const tf2::Matrix3x3& curr_rot = curr_pose.getBasis();
  const tf2::Matrix3x3& goal_rot = goal_pose.getBasis();
  for(size_t i=0; i<3; i++)
  {
    for(size_t j=0; j<3; j++)
    {
      ik_req->goal_pose.push_back(goal_rot[i][j]);
      ik_req->current_pose.push_back(curr_rot[i][j]);
    }
    ik_req->goal_pose.push_back(goal_pos[i]);
    ik_req->current_pose.push_back(curr_pos[i]);
  }
  std::vector<float> v{0, 0, 0, 1};
  ik_req->goal_pose.insert(ik_req->goal_pose.end(), v.begin(), v.end());
  ik_req->current_pose.insert(ik_req->current_pose.end(), v.begin(), v.end());

  ik_req->current_joint.push_back(base_state.position[0]);
  ik_req->current_joint.push_back(base_state.position[1]);
  ik_req->current_joint.push_back(base_state.rotation);

  for(size_t i = 0; i < arm_state.jnt_pos.size(); i++){
    ik_req->current_joint.push_back(arm_state.jnt_pos[i]);
  }

  try
  {
    // std::shared_future<std::shared_ptr<
    //   mm_msgs::srv::IK::Response>> response_future;
    // response_future = mm_ik_client_->async_send_request(ik_req).future.share();
    mm_ik_client_->async_send_request(ik_req,
      [this](const rclcpp::Client<mm_msgs::srv::IK>::SharedFuture future) -> void{
        rt_mm_ik_res_ptr_.writeFromNonRT(future.get());
      }
    );

    // // wait for the service to respond
    // std::chrono::nanoseconds wait_time((period * 0.2).nanoseconds());
    // std::future_status fs = response_future.wait_for(wait_time);
    // if(fs == std::future_status::timeout)
    // {
    //   RCLCPP_WARN(get_node()->get_logger(), "MM IK Service timed out.");
    //   return false;
    // }
    // else
    // {
    //   std::shared_ptr<mm_msgs::srv::IK_Response> res = response_future.get();
    //   if(res->joint_speed.size() == 9)
    //   {
    //     base_state.lin_vel = std::hypot(res->joint_speed[0], res->joint_speed[1]);
    //     base_state.ang_vel = res->joint_speed[2];
    //     if(base_state.lin_vel > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT){
    //       base_state.direction[0] = res->joint_speed[0] / base_state.lin_vel;
    //       base_state.direction[1] = res->joint_speed[1] / base_state.lin_vel;
    //     }else{
    //       base_state.lin_vel = 0;
    //     }
    //     for(unsigned i=0; i < arm_data.goal_state.jnt_vel.size(); i++){
    //       arm_data.goal_state.jnt_vel[i] = res->joint_speed.at(i+3);
    //     }
    //   }else{
    //     RCLCPP_WARN(get_node()->get_logger(), "MM IK Service failed.");
    //     return false;
    //   }
    // }
  }
  catch(const std::exception& e){
    RCLCPP_WARN(get_node()->get_logger(), "Failed to call MM IK service");
    return false;
  }
  return true;
}

bool MMServoController::updateArmCmd(
  mcdd::ArmData& arm_data, const geometry_msgs::msg::Twist& cmd, uint8_t cmd_frame, 
  const tf2::Transform base_offset_trans, const rclcpp::Time & /*time*/, 
  const rclcpp::Duration& period, bool do_ik)
{
  double period_seconds = period.seconds();

  const auto reference_pose = mcu::checkTolerance(
    arm_data.curr_state.pose, arm_data.last_state.pose, 
    arm_data.curr_state.jnt_pos, arm_data.last_state.jnt_pos) ? 
       arm_data.last_state.pose : arm_data.curr_state.pose;
  
  tf2::Vector3 arm_cmd_pos, base_pos_tool, target_pos;
  tf2::Quaternion arm_cmd_quat, base_quat_tool, world_quat_base, target_quat;
  tf2::Transform arm_cmd_trans, base_trans_tool, world_trans_base, target_trans;

  arm_cmd_pos = tf2::Vector3(
    cmd.linear.x * period_seconds, 
    cmd.linear.y * period_seconds, 
    cmd.linear.z * period_seconds
  );
  base_pos_tool = tf2::Vector3(
    reference_pose[0], 
    reference_pose[1], 
    reference_pose[2]
  );
  base_quat_tool = tf2::Quaternion(
    reference_pose[4], 
    reference_pose[5],
    reference_pose[6], 
    reference_pose[3]
  );
  arm_cmd_quat.setRPY(
    cmd.angular.x * period_seconds,
    cmd.angular.y * period_seconds,
    cmd.angular.z * period_seconds
  );
  base_trans_tool.setOrigin(base_pos_tool);
  base_trans_tool.setRotation(base_quat_tool);

  arm_cmd_trans.setIdentity();
  arm_cmd_trans.setOrigin(arm_cmd_pos);

  switch (cmd_frame)
  {
  case mm_msgs::msg::ServoControl::BASE_FRAME:
    target_quat = base_offset_trans.getRotation() * arm_cmd_quat * base_quat_tool;
    target_pos = (base_offset_trans * arm_cmd_trans * base_trans_tool).getOrigin();
    break;
  case mm_msgs::msg::ServoControl::WORLD_FRAME:
    world_quat_base.setRPY(0, 0, base_data_.curr_state.rotation);
    world_trans_base.setIdentity();
    world_trans_base.setRotation(world_quat_base);
    target_quat = base_offset_trans.getRotation() * world_quat_base.inverse() * 
                  arm_cmd_quat * world_quat_base * base_quat_tool;
    target_pos = (base_offset_trans * world_trans_base.inverse() * 
                  arm_cmd_trans * world_trans_base * base_trans_tool).getOrigin();
    break;
  case mm_msgs::msg::ServoControl::TOOL_FRAME:
    target_quat = base_offset_trans.getRotation() * base_quat_tool * arm_cmd_quat;
    target_pos = (base_offset_trans * base_trans_tool * arm_cmd_trans).getOrigin();
    break;
  default:
    RCLCPP_ERROR(get_node()->get_logger(), "Arm command frame %d not support!", cmd_frame);
    return false;
  }
  arm_data.goal_state.pose[0] = target_pos.x();
  arm_data.goal_state.pose[1] = target_pos.y();
  arm_data.goal_state.pose[2] = target_pos.z();
  arm_data.goal_state.pose[3] = target_quat.w();
  arm_data.goal_state.pose[4] = target_quat.x();
  arm_data.goal_state.pose[5] = target_quat.y();
  arm_data.goal_state.pose[6] = target_quat.z();

  if(!do_ik){
    return true;
  }

  if(!mcu::getIKResult(period * 0.3, params_.kinematics.arm_ik_group, 
    params_.kinematics.arm_ik_base_link, arm_data.goal_state.pose, 
    robot_state_, arm_data.goal_state.jnt_pos))
  {
    return false;
  }

  for(size_t i = 0; i < arm_data.joint_names.size(); i++)
  {
    arm_data.goal_state.jnt_vel[i] = 
      (arm_data.goal_state.jnt_pos[i] - arm_data.last_state.jnt_pos[i]) / period_seconds;
    arm_data.goal_state.jnt_acc[i] = 
      (arm_data.goal_state.jnt_vel[i] - arm_data.last_state.jnt_vel[i]) / period_seconds;
  }
  mcu::ensureArmCmdLimit(period, arm_data);

  // update pose cmd to fit joint position cmds
  while(!mcu::getFKResult(arm_data.joint_names, arm_data.next_state.jnt_pos,
    params_.kinematics.arm_ik_base_link, params_.kinematics.arm_ik_end_link,
    robot_state_, arm_data.next_state.pose))
  {
    RCLCPP_WARN(get_node()->get_logger(), 
      "pose cmd update failed, try again!");
  }
  return true;
}

bool MMServoController::calcWheelCmd(const rclcpp::Duration& period, int cnt)
{
  if(!base_kinematics_.inverseKinematics(base_data_, false)){
    return false;
  }
  bool have_to_stop = false;
  bool want_to_stop = true;
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
    return false;
  }
  if(mcu::checkWheelCmdLimit(period, base_data_)){
    std::cout<<"xxx"<<std::endl;
    return true;
  }

  if(++cnt > 10){
    std::cout<<"--------------fuck-------------"<<std::endl;
    return true;
  }
  const double period_seconds = period.seconds();
  auto& ns = base_data_.next_state;
  const auto& ls = base_data_.last_state;
  const double x_acc_diff = 
    0.8 * (ns.acc_direction[0] * ns.dir_diff - ls.acc_direction[0] * ls.dir_diff);
  const double y_acc_diff = 
    0.8 * (ns.acc_direction[1] * ns.dir_diff - ls.acc_direction[1] * ls.dir_diff);
  const double ang_acc_diff = 0.8 * (ns.ang_acc - ls.ang_acc);

  const double acc_factor = (cnt > 5) ? 0.9 : 1;

  const double x_acc = acc_factor * (ls.acc_direction[0] * ls.dir_diff + x_acc_diff);
  const double y_acc = acc_factor * (ls.acc_direction[1] * ls.dir_diff + y_acc_diff);
  const double x_vel = ls.direction[0] * ls.lin_vel + x_acc * period_seconds;
  const double y_vel = ls.direction[1] * ls.lin_vel + y_acc * period_seconds;
  ns.lin_vel = std::hypot(x_vel, y_vel);
  ns.direction[0] = x_vel / ns.lin_vel;
  ns.direction[1] = y_vel / ns.lin_vel;
  ns.dir_diff = std::hypot(x_acc, y_acc);
  ns.acc_direction[0] = x_acc / ns.dir_diff;
  ns.acc_direction[1] = y_acc / ns.dir_diff;
  ns.lin_acc = (ns.lin_vel - ls.lin_vel) / period_seconds;

  ns.ang_acc = acc_factor * (ls.ang_acc + ang_acc_diff);
  ns.ang_vel = ls.ang_vel + ns.ang_acc * period_seconds;

  std::cout<<"---"<<acc_factor<<std::endl;

  return this->calcWheelCmd(period, cnt);
}

bool MMServoController::updateBaseCmd(
  const geometry_msgs::msg::Twist& cmd, const uint8_t cmd_frame,
  const rclcpp::Duration& period)
{
  bool have_to_stop = false;
  bool want_to_stop = true;
  double period_seconds = period.seconds();
  if(sqrt(pow(cmd.linear.x, 2) + pow(cmd.linear.y, 2)) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT
     && fabs(cmd.angular.z) < mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
  {
    this->baseHalt(period);
    return false;
  }
  if(cmd_frame == mm_msgs::msg::ServoControl::BASE_FRAME)
  {
    base_data_.goal_state.lin_vel = std::hypot(cmd.linear.x, cmd.linear.y);
    base_data_.goal_state.ang_vel = cmd.angular.z;
    if(base_data_.goal_state.lin_vel > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
    {
      base_data_.goal_state.direction[0] = cmd.linear.x / base_data_.goal_state.lin_vel;
      base_data_.goal_state.direction[1] = cmd.linear.y / base_data_.goal_state.lin_vel;
    }else{
      base_data_.goal_state.lin_vel = 0;
    }
  }
  else if(cmd_frame == mm_msgs::msg::ServoControl::WORLD_FRAME)
  {
    double sin_theta = sin(base_data_.curr_state.rotation);
    double cos_theta = cos(base_data_.curr_state.rotation);
    double cmd_x = cos_theta * cmd.linear.x + sin_theta * cmd.linear.y;
    double cmd_y = cos_theta * cmd.linear.y - sin_theta * cmd.linear.x;
    base_data_.goal_state.lin_vel = std::hypot(cmd_x, cmd_y);
    base_data_.goal_state.ang_vel = cmd.angular.z;
    if(base_data_.goal_state.lin_vel > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
    {
      base_data_.goal_state.direction[0] = cmd_x / base_data_.goal_state.lin_vel;
      base_data_.goal_state.direction[1] = cmd_y / base_data_.goal_state.lin_vel;
    }else{
      base_data_.goal_state.lin_vel = 0;
    }
  }
  else{
    RCLCPP_ERROR(get_node()->get_logger(), "Base command frame %d not support!", cmd_frame);
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
  
  bool steering_mode = false;
  if(base_data_.is_halted)
  {
    double dis;
    for(auto & wheel_data : base_data_.wheel_data_vector)
    {
      const double st_a_diff = wheel_data->swerve_joint_limits.acc * period_seconds;
      dis = wheel_data->goal_state.st_pos - wheel_data->last_state.st_pos;
      if((fabs(dis) > st_a_diff) || fabs(wheel_data->last_state.st_vel) > st_a_diff)
      {
        steering_mode = true;
        break;
      }
    }
    if(steering_mode)
    {
      double t_max = 0;
      for(auto & wheel_data : base_data_.wheel_data_vector)
      {
        double slope = wheel_data->swerve_joint_limits.acc;
        double v_max = wheel_data->swerve_joint_limits.vel;
        double pos_diff = wheel_data->goal_state.st_pos - wheel_data->last_state.st_pos;
        double v0 = wheel_data->last_state.st_vel;
        double t = mcu::calcMovingTime(pos_diff, v_max, slope, v0, 0);
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
    this->baseHalt(period);
    return false;
  }

  base_data_.next_state.lin_acc = base_data_.dir_acc_max;
  base_data_.next_state.ang_acc = base_data_.ang_acc_max;
  mcu::ensureBaseCmdLimit(
    period, base_data_, base_data_.next_state.lin_acc, base_data_.next_state.ang_acc, 
    base_data_.next_state.direction, base_data_.next_state.lin_vel, base_data_.next_state.ang_vel);
  // if(!base_kinematics_.checkSingularity(base_data_, false)){
  //   RCLCPP_WARN(get_node()->get_logger(), "Base command close to the singularity!");
  // }

  mcdd::Double2 acc_dir_v = {
    base_data_.next_state.direction[0] * base_data_.next_state.lin_vel - 
    base_data_.last_state.direction[0] * base_data_.last_state.lin_vel,
    base_data_.next_state.direction[1] * base_data_.next_state.lin_vel - 
    base_data_.last_state.direction[1] * base_data_.last_state.lin_vel,
  };
  base_data_.next_state.dir_diff = std::hypot(acc_dir_v[0], acc_dir_v[1]);
  base_data_.next_state.acc_direction[0] = acc_dir_v[0] / base_data_.next_state.dir_diff;
  base_data_.next_state.acc_direction[1] = acc_dir_v[1] / base_data_.next_state.dir_diff;
  // base_data_.next_state.lin_acc = 
  //   (base_data_.next_state.lin_vel - base_data_.last_state.lin_vel) / period_seconds;
  // base_data_.next_state.ang_acc =
  //   (base_data_.next_state.ang_vel - base_data_.last_state.ang_vel) / period_seconds;

  // if(!this->calcWheelCmd(period)){
  //   this->baseHalt(period);
  //   return false;
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
    this->baseHalt(period);
    return false;
  }
  mcu::ensureWheelCmdLimit(period, base_data_);

  // cmd FK here is to update the next state of base data affected by wheel limit
  for(auto& wheel_data : base_data_.wheel_data_vector)
  {
    wheel_data->curr_state.velocity = mcu::radsTometers(
      fabs(wheel_data->next_state.dr_vel), wheel_data->radius);
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

void MMServoController::armHalt(mcdd::ArmData& arm_data, const rclcpp::Duration & period)
{
  mcu::armHalt(period, arm_interfaces_[arm_data.arm_name], arm_data);
}

void MMServoController::baseHalt(const rclcpp::Duration & period)
{
  mcu::baseHalt(period, wheel_interfaces_, base_data_);
}
}  // namespace mm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mm_controllers::MMServoController, controller_interface::ControllerInterface)
