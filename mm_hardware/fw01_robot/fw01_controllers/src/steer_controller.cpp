#include "fw01_controllers/steer_controller.hpp"

namespace fw01_controllers
{
  controller_interface::CallbackReturn fw01_controllers::SteerController::on_init()
  {
    //TODO: Space allocate msg vector if need to. otherwise nothing to do here
    //initMsgs();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::SteerController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("steering_ctrl_cmd/steering_ctrl_cmd_gear");
    config.names.emplace_back("steering_ctrl_cmd/steering_ctrl_cmd_velocity");
    config.names.emplace_back("steering_ctrl_cmd/steering_ctrl_cmd_steering");
    config.names.emplace_back("steering_ctrl_cmd/steering_ctrl_cmd_slipangle");

    return config;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::SteerController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("system_interface/initialized");
    
    return config;
  }


  controller_interface::return_type fw01_controllers::SteerController::update(const rclcpp::Time& /*time*/, 
                                                                          const rclcpp::Duration& /*period*/)
  {
    //TODO:
 
    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn
  fw01_controllers::SteerController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }


  controller_interface::CallbackReturn
  fw01_controllers::SteerController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
       RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
       std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    try {
      //register publisher
      set_steer_srv_ = get_node()->create_service<fw01_msgs::srv::SteerCmd>("~/fw01_cmd_steer", std::bind(&SteerController::setSteerCmd, this, std::placeholders::_1, std::placeholders::_2));
      set_transverse_srv_ = get_node()->create_service<fw01_msgs::srv::TransverseCmd>("~/fw01_cmd_transverse", std::bind(&SteerController::setTransverseCmd, this, std::placeholders::_1, std::placeholders::_2));
      stop_robot_srv_ = get_node()->create_service<std_srvs::srv::Trigger>("~/fw01_stop_robot", std::bind(&SteerController::stopRobotTrigger, this, std::placeholders::_1, std::placeholders::_2));    
      set_steering_ctrl_srv_ = get_node()->create_service<fw01_msgs::srv::SteeringCtrlCmd>("~/fw01_steering_ctrl_cmd", std::bind(&SteerController::setSteeringCtrlCmd, this, std::placeholders::_1, std::placeholders::_2));    
      
      cmdvel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", 10, std::bind(&SteerController::msgTwistCallback, this, std::placeholders::_1));
       //TODO:
    } catch (...) {
      RCLCPP_FATAL(get_node()->get_logger(), "Error when creating service publisher");
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  fw01_controllers::SteerController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    try {
      // reset publisher
      set_steer_srv_.reset();
      set_transverse_srv_.reset();
      stop_robot_srv_.reset();
      set_steering_ctrl_srv_.reset();
      cmdvel_sub_.reset();
    } catch (...) {
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  bool SteerController::setSteerCmd(fw01_msgs::srv::SteerCmd::Request::SharedPtr req, fw01_msgs::srv::SteerCmd::Response::SharedPtr resp)
  {
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_GEAR].set_value(static_cast<double> (cmd_gear::steer));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_VELOCITY].set_value(static_cast<double> (req->velocity));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_STEERING].set_value(static_cast<double> (req->angular));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_SLIPANGLE].set_value(static_cast<double> (0));

    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());

    resp->success = true;
    return true;
  }

  bool SteerController::setTransverseCmd(fw01_msgs::srv::TransverseCmd::Request::SharedPtr req, fw01_msgs::srv::TransverseCmd::Response::SharedPtr resp)
  {
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_GEAR].set_value(static_cast<double> (cmd_gear::transverse));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_VELOCITY].set_value(static_cast<double> (req->velocity));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_STEERING].set_value(static_cast<double> (0));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_SLIPANGLE].set_value(static_cast<double> (req->slipangle));
    
    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    resp->success = true;
    return true;
  }

  bool SteerController::setSteeringCtrlCmd(fw01_msgs::srv::SteeringCtrlCmd::Request::SharedPtr req, fw01_msgs::srv::SteeringCtrlCmd::Response::SharedPtr resp)
  {
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_GEAR].set_value(static_cast<double> (req->gear));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_VELOCITY].set_value(static_cast<double> (req->velocity));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_STEERING].set_value(static_cast<double> (req->angular));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_SLIPANGLE].set_value(static_cast<double> (req->slipangle));
    
    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    resp->success = true;
    return true;
  }

  bool SteerController::stopRobotTrigger(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr resp) {

    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_GEAR].set_value(static_cast<double> (cmd_gear::neutral));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_VELOCITY].set_value(static_cast<double> (0));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_STEERING].set_value(static_cast<double> (0));
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_SLIPANGLE].set_value(static_cast<double> (0));
        
    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    resp->success = true;
    return true;    
  }

  void SteerController::msgTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double velx = msg->linear.x;
    double vely = msg->linear.y;
    double velth = msg->angular.z;

    double gear, vel, angular, slipangle;
    if (velth != 0) {
      gear = cmd_gear::steer;
      vel = hypotf64(velx, vely);
      angular = velth * 180.0 / M_PI; // Todo: Verify unit for slip angle is correct
      slipangle = 0; //atan2 (dy, dx)
    }
    else {
      gear = cmd_gear::transverse;
      vel = hypotf64(velx, vely);
      angular = 0;
      slipangle = atan2(vely, velx) * 180.0 / M_PI; //unit in degree. atan2 (dy, dx)
      if ((slipangle < -90.0) || (slipangle > 90.0)) {
        vel *= -1;
        slipangle += (slipangle < -90.0) ? 180 : -180; 
      } 
    } 

//RCLCPP_WARN(get_node()->get_logger(), "Delete: (x,y,th) (%lf, %lf, %lf), (%lf, %lf, %lf, %lf)", velx, vely, velth, gear, vel, angular, slipangle);

    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_GEAR].set_value(gear);
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_VELOCITY].set_value(vel);
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_STEERING].set_value(angular);
    command_interfaces_[CommandInterfaces::STEERING_CTRL_CMD_SLIPANGLE].set_value(slipangle);
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fw01_controllers::SteerController, controller_interface::ControllerInterface)
