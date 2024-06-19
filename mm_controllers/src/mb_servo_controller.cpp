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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mm_controllers/mb_servo_controller.hpp"

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
MBServoController::MBServoController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn MBServoController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    base_kinematics_ = FourWheelSteeringKinematics();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MBServoController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    mcu::initBaseData(params_, base_data_);

    base_kinematics_.configuration(base_data_);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this->get_node());
    odom_pub_ = this->get_node()->create_publisher<
      nav_msgs::msg::Odometry>(DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
    cmd_sub_  = this->get_node()->create_subscription<
      CmdType>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<CmdType> msg) -> void {
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
  }
  catch(const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MBServoController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
  MBServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = 
    controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    command_interfaces_config.names.reserve(
      params_.wheel_names.size() * base_data_.swerve_command_interfaces.size() + 
      params_.wheel_names.size() * base_data_.wheel_command_interfaces.size());

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
  MBServoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  try
  {
    state_interfaces_config.names.reserve(
      params_.wheel_names.size() * base_data_.swerve_state_interfaces.size() + 
      params_.wheel_names.size() * base_data_.wheel_state_interfaces.size());

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

controller_interface::return_type MBServoController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
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

  if(!last_cmd_msg || !(*last_cmd_msg))
  {
    return controller_interface::return_type::OK;
  }

  if(updateBaseCmd((*last_cmd_msg)->twist, 
      mm_msgs::msg::ServoControl::BASE_FRAME, period))
  {
    mcu::setBaseCmd(wheel_interfaces_, base_data_);
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MBServoController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  wheel_interfaces_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MBServoController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_.wheel_names.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MBServoController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MBServoController::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

bool MBServoController::calcWheelCmd(const rclcpp::Duration& period, int cnt)
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

bool MBServoController::updateBaseCmd(
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

void MBServoController::baseHalt(const rclcpp::Duration & period)
{
  mcu::baseHalt(period, wheel_interfaces_, base_data_);
}
}  // namespace mm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mm_controllers::MBServoController, controller_interface::ControllerInterface)
