#include "fw01_controllers/freedrive_controller.hpp"

namespace fw01_controllers
{
  controller_interface::CallbackReturn fw01_controllers::FreedriveController::on_init()
  {
    //TODO: Space allocate msg vector if need to. otherwise nothing to do here
    //initMsgs();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::FreedriveController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("rf_swerve_joint/position");
    config.names.emplace_back("lf_swerve_joint/position");
    config.names.emplace_back("lr_swerve_joint/position");
    config.names.emplace_back("rr_swerve_joint/position");
    config.names.emplace_back("rf_wheel_joint/velocity");
    config.names.emplace_back("lf_wheel_joint/velocity");
    config.names.emplace_back("lr_wheel_joint/velocity");
    config.names.emplace_back("rr_wheel_joint/velocity");
  
    return config;
  }

  controller_interface::InterfaceConfiguration fw01_controllers::FreedriveController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("rf_swerve_joint/position");
    config.names.emplace_back("lf_swerve_joint/position");
    config.names.emplace_back("lr_swerve_joint/position");
    config.names.emplace_back("rr_swerve_joint/position");
    config.names.emplace_back("system_interface/initialized");
    
    return config;
  }


  controller_interface::return_type fw01_controllers::FreedriveController::update(const rclcpp::Time& /*time*/, 
                                                                          const rclcpp::Duration& /*period*/)
  {
    //TODO:
 
    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn
  fw01_controllers::FreedriveController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }


  controller_interface::CallbackReturn
  fw01_controllers::FreedriveController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    while (state_interfaces_[4].get_value() == 0.0) {
       RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
       std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    stopRobot();

    try {
      //register publisher
      //set_freedrive_srv_ = get_node()->create_service<fw01_msgs::srv::FreedriveCmd>("~/freedrive_cmd", std::bind(&FreedriveController::setFreedriveCmd, this, std::placeholders::_1, std::placeholders::_2));
      //set_freedrive_individual_srv_ = get_node()->create_service<fw01_msgs::srv::FreedriveIndividualCmd>("~/freedrive_joint_cmd", std::bind(&FreedriveController::setFreedriveIndividualCmd, this, std::placeholders::_1, std::placeholders::_2));
      stop_robot_srv_ = get_node()->create_service<std_srvs::srv::Trigger>("~/stop_robot", std::bind(&FreedriveController::stopRobotTrigger, this, std::placeholders::_1, std::placeholders::_2));    
          
      freedrive_all_sub_ = get_node()->create_subscription<fw01_msgs::msg::Freedrive>("~/freedrive_robot_cmd", 10, std::bind(&FreedriveController::setCombinedJointCallback, this, std::placeholders::_1));
      freedrive_joint_sub_ = get_node()->create_subscription<fw01_msgs::msg::FreedriveJoint>("~/freedrive_joint_cmd", 10, std::bind(&FreedriveController::setIndividualJointCallback, this, std::placeholders::_1));
        
      //TODO:
    } catch (...) {
      RCLCPP_FATAL(get_node()->get_logger(), "Error when creating service publisher FreedriveController");
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  fw01_controllers::FreedriveController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    try {
      // reset publisher
      //set_freedrive_srv_.reset();
      //set_freedrive_individual_srv_.reset();
      stop_robot_srv_.reset();
      freedrive_all_sub_.reset();
      freedrive_joint_sub_.reset();
    } catch (...) {
      return LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // bool FreedriveController::setFreedriveCmd(fw01_msgs::srv::FreedriveCmd::Request::SharedPtr req, fw01_msgs::srv::FreedriveCmd::Response::SharedPtr resp)
  // {
  //   command_interfaces_[0].set_value(static_cast<double> (req->rf_swerve_pos_rad));
  //   command_interfaces_[1].set_value(static_cast<double> (req->lf_swerve_pos_rad));
  //   command_interfaces_[2].set_value(static_cast<double> (req->lr_swerve_pos_rad));
  //   command_interfaces_[3].set_value(static_cast<double> (req->rr_swerve_pos_rad));
  //   command_interfaces_[4].set_value(static_cast<double> (req->rf_wheel_vel_rad));
  //   command_interfaces_[5].set_value(static_cast<double> (req->lf_wheel_vel_rad));
  //   command_interfaces_[6].set_value(static_cast<double> (req->lr_wheel_vel_rad));
  //   command_interfaces_[7].set_value(static_cast<double> (req->rr_wheel_vel_rad));

  //   // TODO: You can also wait until update done here.
  //   // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
  //   //   // Asynchronous wait until the hardware interface has set the io value
  //   //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //   // }
  //   //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());

  //   resp->success = true;
  //   return resp->success;
  // }

  // bool FreedriveController::setFreedriveIndividualCmd(fw01_msgs::srv::FreedriveIndividualCmd::Request::SharedPtr req, fw01_msgs::srv::FreedriveIndividualCmd::Response::SharedPtr resp)
  // {
  //   if ((req->quad_id) >=4) {
  //     return false;
  //   }

  //   command_interfaces_[static_cast<int> (req->quad_id)].set_value(static_cast<double> (req->swerve_pos_rad));
  //   command_interfaces_[(static_cast<int> (req->quad_id))+4].set_value(static_cast<double> (req->wheel_vel_rad));

  //   // TODO: You can also wait until update done here.
  //   // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
  //   //   // Asynchronous wait until the hardware interface has set the io value
  //   //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //   // }
  //   //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
  //   resp->success = true;
  //   return resp->success;
  // }

  bool FreedriveController::stopRobotTrigger(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr resp) {

    stopRobot();
        
    // TODO: You can also wait until update done here.
    // while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    //   // Asynchronous wait until the hardware interface has set the io value
    //   std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // }
    //resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    resp->success = true;
    return resp->success;    
  }

  void FreedriveController::stopRobot(void) {
    for (uint8_t j=0; j<4; j++) {
      command_interfaces_[j].set_value(state_interfaces_[j].get_value());
      command_interfaces_[j+4].set_value(static_cast<double> (0));
    }
  }

  void FreedriveController::setCombinedJointCallback(const fw01_msgs::msg::Freedrive::SharedPtr msg) {
    command_interfaces_[0].set_value(static_cast<double> (msg->rf_swerve_pos_rad));
    command_interfaces_[1].set_value(static_cast<double> (msg->lf_swerve_pos_rad));
    command_interfaces_[2].set_value(static_cast<double> (msg->lr_swerve_pos_rad));
    command_interfaces_[3].set_value(static_cast<double> (msg->rr_swerve_pos_rad));
    command_interfaces_[4].set_value(static_cast<double> (msg->rf_wheel_vel_rad));
    command_interfaces_[5].set_value(static_cast<double> (msg->lf_wheel_vel_rad));
    command_interfaces_[6].set_value(static_cast<double> (msg->lr_wheel_vel_rad));
    command_interfaces_[7].set_value(static_cast<double> (msg->rr_wheel_vel_rad));
  }

  void FreedriveController::setIndividualJointCallback(const fw01_msgs::msg::FreedriveJoint::SharedPtr msg) {
    uint8_t id = msg->quad_id;
    command_interfaces_[id].set_value(static_cast<double> (msg->swerve_pos_rad));
    command_interfaces_[(id+4)].set_value(static_cast<double> (msg->wheel_vel_rad));
  }


}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fw01_controllers::FreedriveController, controller_interface::ControllerInterface)
