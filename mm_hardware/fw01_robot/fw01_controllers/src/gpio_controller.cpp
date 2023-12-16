#include "fw01_controllers/gpio_controller.hpp"

namespace fw01_controllers
{
  controller_interface::CallbackReturn fw01_controllers::GPIOController::on_init()
  {
    //TODO: Space allocate msg vector if need to. otherwise nothing to do here
    //initMsgs();
    battMinCapacity_ = 5; // Unit in Ah. TODO: Read this value from controller yaml instead
    battMaxCapacity_ = 30;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::GPIOController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("io_cmd/io_cmd_lamp_ctrl");
    config.names.emplace_back("io_cmd/io_cmd_unlock");
    config.names.emplace_back("io_cmd/io_cmd_lower_beam_headlamp");
    config.names.emplace_back("io_cmd/io_cmd_left_turn_lamp");
    config.names.emplace_back("io_cmd/io_cmd_right_turn_lamp");
    config.names.emplace_back("io_cmd/io_cmd_braking_lamp");

    return config;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::GPIOController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    //Add interface for joint states
    config.names.emplace_back("rf_swerve_joint/position");
    config.names.emplace_back("rf_swerve_joint/velocity");
    config.names.emplace_back("rf_wheel_joint/position");
    config.names.emplace_back("rf_wheel_joint/velocity");
    config.names.emplace_back("lf_swerve_joint/position");
    config.names.emplace_back("lf_swerve_joint/velocity");
    config.names.emplace_back("lf_wheel_joint/position");
    config.names.emplace_back("lf_wheel_joint/velocity");
    config.names.emplace_back("lr_swerve_joint/position");
    config.names.emplace_back("lr_swerve_joint/velocity");
    config.names.emplace_back("lr_wheel_joint/position");
    config.names.emplace_back("lr_wheel_joint/velocity");
    config.names.emplace_back("rr_swerve_joint/position");
    config.names.emplace_back("rr_swerve_joint/velocity");
    config.names.emplace_back("rr_wheel_joint/position");
    config.names.emplace_back("rr_wheel_joint/velocity");

    config.names.emplace_back("io_fb/io_fb_lamp_ctrl");
    config.names.emplace_back("io_fb/io_fb_unlock");
    config.names.emplace_back("io_fb/io_fb_lower_beam_headlamp");
    config.names.emplace_back("io_fb/io_fb_left_turn_lamp");
    config.names.emplace_back("io_fb/io_fb_right_turn_lamp");
    config.names.emplace_back("io_fb/io_fb_braking_lamp");
    config.names.emplace_back("steering_ctrl_fb/steering_ctrl_fb_gear");
    config.names.emplace_back("steering_ctrl_fb/steering_ctrl_fb_velocity");
    config.names.emplace_back("steering_ctrl_fb/steering_ctrl_fb_steering");
    config.names.emplace_back("steering_ctrl_fb/steering_ctrl_fb_slipangle");
    //config.names.emplace_back("lf_swerve_joint/position");   // config.names.emplace_back("bms_fb/bms_fb_voltage");
    //config.names.emplace_back("rf_swerve_joint/position");
    config.names.emplace_back("bms_fb/bms_fb_voltage"); //config.names.emplace_back("lf_swerve_joint/position"); 
    config.names.emplace_back("bms_fb/bms_fb_current");
    config.names.emplace_back("bms_fb/bms_fb_remaining_capacity");
    config.names.emplace_back("system_interface/initialized");

    return config;
  }


  controller_interface::return_type fw01_controllers::GPIOController::update(const rclcpp::Time& /*time*/, 
                                                                          const rclcpp::Duration& /*period*/)
  {
    //TODO:
    publishIOFb();
    publishBattFb();
    publishRobotStates();

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn
  fw01_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void GPIOController::publishIOFb(void)
  {
    io_msg_.io_fb_lamp_ctrl = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_LAMP_CTRL].get_value());
    io_msg_.io_fb_unlock = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_UNLOCK].get_value());
    io_msg_.io_fb_lower_beam_headlamp = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_LOWER_BEAM_HEADLAMP].get_value());
    io_msg_.io_fb_left_turn_lamp = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_LEFT_TURN_LAMP].get_value());
    io_msg_.io_fb_right_turn_lamp = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_RIGHT_TURN_LAMP].get_value());
    io_msg_.io_fb_braking_lamp = static_cast<bool>(state_interfaces_[StateInterfaces::IO_FB_BRAKING_LAMP].get_value());
    io_pub_->publish(io_msg_);
  }

  void GPIOController::publishBattFb(void)
  {
    battery_msg_.voltage = static_cast<double>(state_interfaces_[StateInterfaces::BMS_FB_VOLTAGE].get_value());
    battery_msg_.current = static_cast<double>(state_interfaces_[StateInterfaces::BMS_FB_CURRENT].get_value());
    double cap = static_cast<double>(state_interfaces_[StateInterfaces::BMS_FB_REMAINING_CAPACITY].get_value());
    if (cap < battMinCapacity_)
      cap = battMinCapacity_;
    if (cap > battMaxCapacity_)
      cap = battMaxCapacity_;
    battery_msg_.percent = ((cap - battMinCapacity_) / (battMaxCapacity_ - battMinCapacity_)) * 100;
    battery_pub_->publish(battery_msg_);
  }

  void GPIOController::publishRobotStates(void)
  {
    robot_msg_.status = 1;
    for (int j=0; j<4; j++) {
      robot_msg_.swerve_pos_rad[j] = state_interfaces_[4*j].get_value();
      robot_msg_.swerve_vel_rad[j] = state_interfaces_[(4*j)+1].get_value();
      robot_msg_.wheel_pos_rad[j] = state_interfaces_[(4*j)+2].get_value();
      robot_msg_.wheel_vel_rad[j] = state_interfaces_[(4*j)+3].get_value();
    }
    robot_pub_->publish(robot_msg_);
  }

  controller_interface::CallbackReturn
  fw01_controllers::GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
       RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
       std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    setDefaultIO();

    try {
      //register publisher
      io_pub_ = get_node()->create_publisher<fw01_msgs::msg::IOStates>("~/io_fb", rclcpp::SystemDefaultsQoS());
      battery_pub_ = get_node()->create_publisher<fw01_msgs::msg::BatteryStates>("~/battery_data", rclcpp::SystemDefaultsQoS());
      robot_pub_ = get_node()->create_publisher<fw01_msgs::msg::RobotStates>("~/joint_data", rclcpp::SystemDefaultsQoS());
      set_io_srv_ = get_node()->create_service<fw01_msgs::srv::IOCmd>("~/io_cmd", std::bind(&GPIOController::setIOCmd, this, std::placeholders::_1, std::placeholders::_2));
      //TODO:
    } catch (...) {
      RCLCPP_FATAL(get_node()->get_logger(), "Error when creating service publisher");
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  fw01_controllers::GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    try {
      // reset publisher
      io_pub_.reset();
      battery_pub_.reset();
      robot_pub_.reset();
      set_io_srv_.reset();
    } catch (...) {
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  bool GPIOController::setIOCmd(fw01_msgs::srv::IOCmd::Request::SharedPtr req, fw01_msgs::srv::IOCmd::Response::SharedPtr resp)
  {
    command_interfaces_[CommandInterfaces::IO_CMD_LAMP_CTRL].set_value(static_cast<double> (req->io_cmd_lamp_ctrl));
    command_interfaces_[CommandInterfaces::IO_CMD_UNLOCK].set_value(static_cast<double> (req->io_cmd_unlock));
    command_interfaces_[CommandInterfaces::IO_CMD_LOWER_BEAM_HEADLAMP].set_value(static_cast<double> (req->io_cmd_lower_beam_headlamp));
    command_interfaces_[CommandInterfaces::IO_CMD_LEFT_TURN_LAMP].set_value(static_cast<double> (req->io_cmd_left_turn_lamp));
    command_interfaces_[CommandInterfaces::IO_CMD_RIGHT_TURN_LAMP].set_value(static_cast<double> (req->io_cmd_right_turn_lamp));
    command_interfaces_[CommandInterfaces::IO_CMD_BRAKING_LAMP].set_value(static_cast<double> (req->io_cmd_braking_lamp));

    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    resp->success = true;
    return resp->success;
  }


  void GPIOController::setDefaultIO(void) // default
  {
    command_interfaces_[CommandInterfaces::IO_CMD_LAMP_CTRL].set_value(1);
    command_interfaces_[CommandInterfaces::IO_CMD_UNLOCK].set_value(0);
    command_interfaces_[CommandInterfaces::IO_CMD_LOWER_BEAM_HEADLAMP].set_value(0);
    command_interfaces_[CommandInterfaces::IO_CMD_LEFT_TURN_LAMP].set_value(0);
    command_interfaces_[CommandInterfaces::IO_CMD_RIGHT_TURN_LAMP].set_value(0);
    command_interfaces_[CommandInterfaces::IO_CMD_BRAKING_LAMP].set_value(0);

 //RCLCPP_INFO(get_node()->get_logger(), "Switch Off Light");

    //command_interfaces_[CommandInterfaces::IO_CMD_LAMP_CTRL].set_value(0);
    //command_interfaces_[CommandInterfaces::IO_CMD_UNLOCK].set_value(0);
    //command_interfaces_[CommandInterfaces::IO_CMD_LOWER_BEAM_HEADLAMP].set_value(0);
    //command_interfaces_[CommandInterfaces::IO_CMD_LEFT_TURN_LAMP].set_value(0);
    //command_interfaces_[CommandInterfaces::IO_CMD_RIGHT_TURN_LAMP].set_value(0);
    //command_interfaces_[CommandInterfaces::IO_CMD_BRAKING_LAMP].set_value(0);
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fw01_controllers::GPIOController, controller_interface::ControllerInterface)

