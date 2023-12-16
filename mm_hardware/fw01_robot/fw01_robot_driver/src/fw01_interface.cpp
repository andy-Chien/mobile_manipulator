#include "fw01_robot_driver/fw01_interface.hpp"

#include <math.h>
#include <rclcpp/logging.hpp>

namespace fw01_robot_driver
{
  static const std::string HW_NAME = "BaseHardwareFW01";

  hardware_interface::CallbackReturn
  BaseHardwareFW01::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    //info_ = info; // This is not required as info_ is initialized in above parent's on_init(info)

    // Allocate memory, Read params, Display Params, Check Joint Size, Interface Name, and other URDF settings, finally Connect
    // init instance variables, params, vectors. TOTRY:
    //actual_io_cmd_bits_copy_.resize(17, std::numeric_limits<double>::quiet_NaN()); // No need to resize. declared as 17 at beginning
    //actual_io_fb_bits_copy_.resize(43, std::numeric_limits<double>::quiet_NaN()); // No need to resize. declared as 43 at beginning
    //actual_bms_fb_bits_copy_.resize(3, std::numeric_limits<double>::quiet_NaN()); // No need to resize. declared as 3 at beginning

    hw_swerve_commands_position_.resize(4, std::numeric_limits<double>::quiet_NaN());
    hw_swerve_states_position_.resize(4, std::numeric_limits<double>::quiet_NaN());
    hw_swerve_states_velocity_.resize(4, std::numeric_limits<double>::quiet_NaN());

    hw_wheel_commands_velocity_.resize(4, std::numeric_limits<double>::quiet_NaN());
    hw_wheel_states_position_.resize(4, std::numeric_limits<double>::quiet_NaN());
    hw_wheel_states_velocity_.resize(4, std::numeric_limits<double>::quiet_NaN());
    hw_last_wheel_enc_count_.resize(4, std::numeric_limits<int32_t>::quiet_NaN());

    if (info_.joints.size() != 8) {
      RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Hardware FW01 urdf has %d Joints. 8 expected.!", (int) info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Joint '%s' has %zu command interfaces. 1 expected.!", joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      //if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
      //      joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
      if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Joint '%s' has %s command interface. Expected %s or %s.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(),        
          hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Joint '%s' has %zu states interfaces. 2 expected.!", joint.name.c_str(), joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
            joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY))
      {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Joint '%s' has %s state interface. Expected %s or %s.", joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  //
  std::vector<hardware_interface::StateInterface> BaseHardwareFW01::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // TOCHECK: Check value of state interface, and whether it is okay to state both POSITION and VELOCITY
    // TOCHECK: Check joint states, below tied correctly AND VARIABLE NAME
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // Quadrant conventions
      if (i%2) { // odd. Wheel (1,3,5,7)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_wheel_states_position_[(i-1)/2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_states_velocity_[(i-1)/2]));
//RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Delete this. Name: %s, index %ld", info_.joints[i].name.c_str(), (i-1)/2);          
      } 
      else { // even. Swerve (0,2,4,6)
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_swerve_states_position_[i/2]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_swerve_states_velocity_[i/2]));
//RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Delete this. Name: %s, index %ld", info_.joints[i].name.c_str(), i/2);   
      }
    }

    // GPIOs
    for (auto& gpio : info_.gpios) {
      if (gpio.name == "io_fb") {
        //for (uint j = 0; j < gpio.state_interfaces.size(); ++j) {
        //  state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, gpio.state_interfaces[j].name, &actual_io_fb_bits_copy_[std::stoi(gpio.state_interfaces[j].bit)]));
        //}

        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_lamp_ctrl" , &actual_io_fb_bits_copy_[0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_unlock", &actual_io_fb_bits_copy_[1]));
        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_lower_beam_headlamp", &actual_io_fb_bits_copy_[8]));
        //state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_upper_beam_headlamp", &actual_io_fb_bits_copy_[9]));
        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_left_turn_lamp", &actual_io_fb_bits_copy_[10]));
        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_right_turn_lamp", &actual_io_fb_bits_copy_[11]));
        state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_braking_lamp", &actual_io_fb_bits_copy_[12]));
        //state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_clearance_lamp", &actual_io_fb_bits_copy_[13]));
        //state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_fog_lamp", &actual_io_fb_bits_copy_[14]));
        //state_interfaces.emplace_back(hardware_interface::StateInterface("io_fb", "io_fb_speaker", &actual_io_fb_bits_copy_[16]));
      }
      else if (gpio.name == "steering_ctrl_fb") {
        state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, gpio.state_interfaces[0].name, &hw_steering_ctrl_fb_gear_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, gpio.state_interfaces[1].name, &hw_steering_ctrl_fb_velocity_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, gpio.state_interfaces[2].name, &hw_steering_ctrl_fb_steering_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(gpio.name, gpio.state_interfaces[3].name, &hw_steering_ctrl_fb_slipangle_));          
      }
      else if (gpio.name.find("_cmd") != std::string::npos) {
      }
      else {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Shouldn't reach here. Failed to compare gpio.name (%s) as io_fb", gpio.name.c_str()); 
      }      
    }

    // Sensors
    for (auto& sensor : info_.sensors) {
      if (sensor.name == "bms_fb") {
        for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
          state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name, &actual_bms_fb_bits_copy_[j]));
        }
      } else if (sensor.name == "bms_flag_fb") {

      }
      else {
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Shouldn't reach here. Failed to compare sensor.name (%s) as bms_fb or bms_flag_fb", sensor.name.c_str()); 
      }
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface("system_interface", "initialized", &hw_system_interface_initialized_));

    return state_interfaces;
  }
	 	
  std::vector<hardware_interface::CommandInterface> BaseHardwareFW01::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      // Quadrant conventions
      if (i%2) { // odd. Wheel (1,3,5,7)
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_wheel_commands_velocity_[(i-1)/2]));
      } 
      else { // even. Swerve (0,2,4,6)
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_swerve_commands_position_[i/2]));
      }
    }

    for (auto& gpio : info_.gpios) {
      if (gpio.name == "io_cmd") {
        //for (uint j = 0; j < gpio.command_interfaces.size(); ++j) {
        //  command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, gpio.state_interfaces[j].name, &actual_io_cmd_bits_copy_[std::stoi(gpio.state_interfaces[j].bit)]));
        //}

        // Hardcode. No need to use value inside xacro
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_lamp_ctrl", &actual_io_cmd_bits_copy_[0]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_unlock", &actual_io_cmd_bits_copy_[1]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_lower_beam_headlamp", &actual_io_cmd_bits_copy_[8]));
        //command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_upper_beam_headlamp", &actual_io_cmd_bits_copy_[9]))
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_left_turn_lamp", &actual_io_cmd_bits_copy_[10]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_right_turn_lamp", &actual_io_cmd_bits_copy_[11]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_braking_lamp", &actual_io_cmd_bits_copy_[12]));
        //command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_clearance_lamp", &actual_io_cmd_bits_copy_[13]));
        //command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_fog_lamp", &actual_io_cmd_bits_copy_[14]));
        //command_interfaces.emplace_back(hardware_interface::CommandInterface("io_cmd", "io_cmd_speaker", &actual_io_cmd_bits_copy_[16]));
      }
      else if (gpio.name == "steering_ctrl_cmd") {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, gpio.command_interfaces[0].name, &hw_steering_ctrl_cmd_gear_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, gpio.command_interfaces[1].name, &hw_steering_ctrl_cmd_velocity_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, gpio.command_interfaces[2].name, &hw_steering_ctrl_cmd_steering_));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(gpio.name, gpio.command_interfaces[3].name, &hw_steering_ctrl_cmd_slipangle_));
      }
      else if (gpio.name.find("_cmd") != std::string::npos) { // There are other _cmd gpios that are not proocessed
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Shouldn't reach here. Unprocessed command_interfaces for gpio (%s)", gpio.name.c_str()); 
      }    
    }

    return command_interfaces;
  }


  BaseHardwareFW01::InterfaceMode BaseHardwareFW01::verify_interface_mode (const std::vector<std::string> & interfaces)  {
    int numSwerveCmds = 0;
    int numWheelCmds = 0;
    int numSteerCmds = 0;

    for (const auto& interface : interfaces) {
      if (interface.find("swerve_joint/position") != std::string::npos) {
        numSwerveCmds++;
      }
      if (interface.find("wheel_joint/velocity") != std::string::npos) {
        numWheelCmds++; 
      }
      if ((interface.find("steering_ctrl_cmd/steering_ctrl_cmd_gear") != std::string::npos) ||
          (interface.find("steering_ctrl_cmd/steering_ctrl_cmd_velocity") != std::string::npos) ||
          (interface.find("steering_ctrl_cmd/steering_ctrl_cmd_steering") != std::string::npos) ||
          (interface.find("steering_ctrl_cmd/steering_ctrl_cmd_slipangle") != std::string::npos)) {
        numSteerCmds++; 
      }
    }

    if ((numSwerveCmds == 0) && (numWheelCmds == 0) && (numSteerCmds == 0)) {
      return BaseHardwareFW01::InterfaceMode::UNINITIALIZED;
    }
    if ((numSwerveCmds == 0) && (numWheelCmds == 0) && (numSteerCmds == 4)) {
      return BaseHardwareFW01::InterfaceMode::STEERCTRL;
    }
    if ((numSwerveCmds == 4) && (numWheelCmds == 4) && (numSteerCmds == 0)) {
      return BaseHardwareFW01::InterfaceMode::FREECTRL;
    }
    // Should not reach here
    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Invalid interface mode-> numSwerveCmds: %d, numWheelCmds: %d, numSteerCmds %d", numSwerveCmds, numWheelCmds, numSteerCmds); 
    return BaseHardwareFW01::InterfaceMode::INVALID;
  }


  hardware_interface::return_type BaseHardwareFW01::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
  {  
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    BaseHardwareFW01::InterfaceMode stopMode = verify_interface_mode(stop_interfaces);
    BaseHardwareFW01::InterfaceMode startMode = verify_interface_mode(start_interfaces);

    if (stopMode == BaseHardwareFW01::InterfaceMode::INVALID) { // Don't care about stop interfaces. Just give warning
      RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Prepare_Switching: INVALID stop_interfaces");
    }

    switch (startMode) {
      case BaseHardwareFW01::InterfaceMode::UNINITIALIZED:
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Prepare_Switching: revert to UNINITIALIZED");
        break;
      case BaseHardwareFW01::InterfaceMode::STEERCTRL:
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Prepare_Switching: activating Steer Ctrl Mode (OEM mode)");
        break;
      case BaseHardwareFW01::InterfaceMode::FREECTRL:
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Prepare_Switching: activating Free Drive Ctrl Mode");
        break;
      case BaseHardwareFW01::InterfaceMode::INVALID:  default:
        RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Prepare_Switching: INVALID start_interfaces");
        ret_val = hardware_interface::return_type::ERROR;     
        break;
    }
    return ret_val;
  }
    
  hardware_interface::return_type BaseHardwareFW01::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
  {
    hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

    BaseHardwareFW01::InterfaceMode stopMode = verify_interface_mode(stop_interfaces);
    BaseHardwareFW01::InterfaceMode startMode = verify_interface_mode(start_interfaces);

        RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Perform_Switching: Delete This");

    // TODO (Future): Populate this during testing. Easiest way is to set wheel command to 0, swerve cmd to 90 degrees when changing mode. 
    // Ideal way to check switch type. If (steer->freedrive), then read all current angle of 4 swerve joint and input as free command. Wheel velocity all zeros
    // if (freedrive > steer). Recompute 4 swerve angle to get robot tv/rv/sv and set as coomand. Wheel velocity are zeros.
    // But just choose easy way probably sufficient

    send_zeros_command(stopMode);

    switch (startMode) {
      case BaseHardwareFW01::InterfaceMode::UNINITIALIZED:
        hw_interface_mode_ = startMode;
        break;
      case BaseHardwareFW01::InterfaceMode::STEERCTRL:
        hw_interface_mode_ = startMode;
        break;
      case BaseHardwareFW01::InterfaceMode::FREECTRL:
        hw_interface_mode_ = startMode;
        break;
      case BaseHardwareFW01::InterfaceMode::INVALID:  default:
        hw_interface_mode_ = BaseHardwareFW01::InterfaceMode::UNINITIALIZED;
                RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Perform_Switching: return error");
        ret_val = hardware_interface::return_type::ERROR;     
        break;
    }
    return ret_val;
  }

  void BaseHardwareFW01::send_zeros_command (void) {
    send_zeros_command(hw_interface_mode_);
  }

  void BaseHardwareFW01::send_zeros_command (fw01_robot_driver::BaseHardwareFW01::InterfaceMode mode) {
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "send zero Delete this %d", mode);
    if (mode == BaseHardwareFW01::InterfaceMode::STEERCTRL) {
      hw_steering_ctrl_cmd_velocity_ = 0;
      write_steer_cmd();
    }
    else if (mode == BaseHardwareFW01::InterfaceMode::FREECTRL) {
      for (int j=0; j<4; j++) {
        hw_wheel_commands_velocity_[j] = 0;
      }
      write_freectrl_cmd();
    } else {
      hw_steering_ctrl_cmd_velocity_ = 0;
      for (int j=0; j<4; j++) {
        hw_wheel_commands_velocity_[j] = 0;
      }
    }
  }

  hardware_interface::CallbackReturn BaseHardwareFW01::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    first_read_pass_ = first_write_pass_ = true;
    set_hw_read_freq(std::stod(info_.hardware_parameters["hw_read_freq"]));
    set_hw_write_freq(std::stod(info_.hardware_parameters["hw_write_freq"]));
    if (std::stod(info_.hardware_parameters["hw_wheel_diam"]) != 0.24) {
      RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "ERROR: Wheel Diameter is not 0.24 meter. Revert back to 0.24m");
    }
    hw_wheel_diam_ = 0.24;
    can_port_ = info_.hardware_parameters["can_port"];

    //RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "[Setting] CAN Port: %s", can_port_.c_str());
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "[Setting] Write_HW_Freq: %.1lf Hz", hw_write_freq_);
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "[Setting] Read_HW_Freq : %.1lf Hz", hw_read_freq_);

    hw_steering_ctrl_cmd_gear_ = 0;
    hw_steering_ctrl_cmd_velocity_ = 0;
    hw_steering_ctrl_cmd_steering_ = 0;
    hw_steering_ctrl_cmd_slipangle_ = 0;
    hw_steering_ctrl_fb_gear_ = 0;
    hw_steering_ctrl_fb_velocity_ = 0;
    hw_steering_ctrl_fb_steering_ = 0;
    hw_steering_ctrl_fb_slipangle_ = 0;
    for(int j=0; j<4; j++) {
      hw_last_wheel_enc_count_[j] = 0;
    }
    hw_interface_mode_ = BaseHardwareFW01::InterfaceMode::UNINITIALIZED;
  
    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "FW01 mode is currently set READ-ONLY"); // Can delete this warning after testing

    hw_system_interface_initialized_ = false;

    // Need to check that bit/id in urdf has to be < 17, <43 and <3. But no one should modify the urdf so assume true
    try {
      RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Connecting to mobile base at \"%s\" ...", can_port_.c_str());
      yhs_can_ = std::make_unique<fw01_robot_driver::YhsCanControl>(can_port_, std::make_shared<rclcpp::Logger>(rclcpp::get_logger(HW_NAME)));
    } catch (...) {
      RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Could not connect to mobile base");
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Successfully connected to mobile base");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseHardwareFW01::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "[On_deactivate] FW01 closing connection...");
    yhs_can_->closeConnection();
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "FW01 Connection Closed!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseHardwareFW01::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "[On_Error] FW01 closing connection...");
    yhs_can_->closeConnection();
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "FW01 Connection Closed!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn BaseHardwareFW01::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "[On_Shutdown] FW01 closing connection...");
    yhs_can_->closeConnection();
    RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "FW01 Connection Closed!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }



  hardware_interface::return_type BaseHardwareFW01::read(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    rclcpp::Duration unused = period;
    if (first_read_pass_ || (time - last_read_time_) > *hw_read_period_) {
      yhs_can_->processUpdates();

      // Joint pos and velocity are in radians.
      double dt, new_val, denc;
      int16_t swerve_pos_count[4];
      int16_t wheel_vel_count[4];
      int32_t wheel_enc_count[4];
      yhs_can_->readJoints(swerve_pos_count, wheel_vel_count, wheel_enc_count); // 1024 per wheel resolution
      if (first_read_pass_) {
        for (uint j=0 ; j<4; j++) {
          hw_swerve_states_velocity_[j] = 0;
          hw_swerve_states_position_[j] =  (swerve_pos_count[j] / 100.0) * M_PI / 180.0 ;

          hw_wheel_states_velocity_[j] = (wheel_vel_count[j] / 1000.0) * 2  / hw_wheel_diam_; // convert m/s to radians
          hw_wheel_states_position_[j] = 0;
          
          hw_last_wheel_enc_count_[j] = wheel_enc_count[j];
        }
      } else {
        dt = (time - last_read_time_).seconds();
        for (uint j=0 ; j<4; j++) {
          new_val = (swerve_pos_count[j] / 100.0) * M_PI / 180.0 ; // swerve;
          hw_swerve_states_velocity_[j] = (new_val - hw_swerve_states_position_[j]) / dt;
          hw_swerve_states_position_[j] = new_val;
          
          new_val = (wheel_vel_count[j] / 1000.0) * 2 / hw_wheel_diam_; // convert m/s to radians
          hw_wheel_states_velocity_[j] = new_val;
          //hw_wheel_states_position_[j] = hw_wheel_states_position_[j] + (new_val * dt); // p from velocity

          denc = wheel_enc_count[j] - hw_last_wheel_enc_count_[j];
          if ((denc > 999999) || (denc < (-999999))) {
            denc = 0; //skip
          }
          hw_wheel_states_position_[j] = hw_wheel_states_position_[j] + (denc * M_PI / 2048); // p from encoder
          hw_last_wheel_enc_count_[j] = wheel_enc_count[j];

          //CLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "Delete enc [%d]", wheel_enc_count[2]);
        }

        // First Time<->subsequent p-v from velocity
        //hw_wheel_states_velocity_[j] = (wheel_vel_count[j] / 1000.0) * 2  / hw_wheel_diam_; // convert m/s to radians
        //hw_wheel_states_position_[j] = 0;
        //  new_val = (wheel_vel_count[j] / 1000.0) * 2 / hw_wheel_diam_; // convert m/s to radians
        //  hw_wheel_states_velocity_[j] = new_val;
        //  hw_wheel_states_position_[j] = hw_wheel_states_position_[j] + (new_val * dt);

        // First Time<->subsequent p from encoder, v from velocity
        //hw_wheel_states_velocity_[j] = (wheel_vel_count[j] / 1000.0) * 2  / hw_wheel_diam_; // convert m/s to radians
        //hw_wheel_states_position_[j] = 0;
        //  new_val = (wheel_vel_count[j] / 1000.0) * 2 / hw_wheel_diam_; // convert m/s to radians
        //  hw_wheel_states_velocity_[j] = new_val;
        //  hw_wheel_states_position_[j] = hw_wheel_states_position_[j] + (new_val * dt);

        // static int i = 0;
        // if (i++ >= 1) {
        //   RCLCPP_INFO(rclcpp::get_logger("BaseHardwareFW01"), "Wheel Pos [%lf, %lf, %lf, %lf], [%lf, %lf, %lf, %lf]", hw_swerve_states_position_[0], hw_swerve_states_position_[1], hw_swerve_states_position_[2], hw_swerve_states_position_[3], hw_wheel_states_velocity_[0], hw_wheel_states_velocity_[1] , hw_wheel_states_velocity_[2] ,hw_wheel_states_velocity_[3]);
        //   i = 0;
        // }
      }

      // io_fb
      bool res;
      for (unsigned int j=0; j<43; j++) {
        if (yhs_can_->readIO(j, res)) { // only for valid index
          actual_io_fb_bits_copy_[j] = (double) res;
        }
      }

      // bms_fb
      uint16_t volt; // 0.01V /bit
			int16_t  cur; // 0.01A /bit
			uint16_t cap; // 0.01Ah /bit 
      yhs_can_->readBMS(volt, cur, cap);
      actual_bms_fb_bits_copy_[0] = (double) (volt/100.0);
      actual_bms_fb_bits_copy_[1] = (double) (cur/100.0);
      actual_bms_fb_bits_copy_[2] = (double) (cap/100.0);

      hw_system_interface_initialized_ = (yhs_can_->initialized_) ? 1.0 : 0.0;

      first_read_pass_ = false;
      last_read_time_ = time;
    }
    return hardware_interface::return_type::OK;
  }

	void BaseHardwareFW01::write_IOs(void) {
    for (int j=0; j<17; j++) {
      yhs_can_->setIO(j, (bool) actual_io_cmd_bits_copy_[j]);
    }
    yhs_can_->writeIOs();
  }

  void BaseHardwareFW01::write_steer_cmd(void) {
    int16_t vel = (int16_t) (hw_steering_ctrl_cmd_velocity_ * 1000);
    int16_t angular = (int16_t) (hw_steering_ctrl_cmd_steering_ * 100);; 
    int16_t slipangle = (int16_t) (hw_steering_ctrl_cmd_slipangle_ * 100);;

//RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Write delete this. %d, %d, %d", vel, angular, slipangle);
    yhs_can_->writeJoints((uint8_t) hw_steering_ctrl_cmd_gear_, vel, angular, slipangle);
  }

  void BaseHardwareFW01::write_freectrl_cmd(void) {
    std::array<int16_t, 4> swerve_cmd;
    std::array<int16_t, 4> wheel_cmd;
    for (int j=0; j<4; j++) {
      swerve_cmd[j] = (int16_t) ((hw_swerve_commands_position_[j] * 180.0 / M_PI) * 100);// Convert radian to angle, then to count
      wheel_cmd[j] = (int16_t) ((hw_wheel_commands_velocity_[j] * hw_wheel_diam_ / 2) * 1000); // convert rad/s to m/s, then to count.
    }
    yhs_can_->writeJoints(swerve_cmd, wheel_cmd);  
  }

	hardware_interface::return_type BaseHardwareFW01::write(const rclcpp::Time &time, const rclcpp::Duration &period) 
  {
    rclcpp::Duration unused = period;
    if (first_write_pass_ || (time - last_read_time_) > *hw_write_period_) {
      first_write_pass_ = false;
//RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Write delete this");
      switch (hw_interface_mode_) {
        case BaseHardwareFW01::InterfaceMode::UNINITIALIZED:
          write_IOs();
          break;
        case BaseHardwareFW01::InterfaceMode::STEERCTRL:
          write_steer_cmd();
          write_IOs();
          break;
        case BaseHardwareFW01::InterfaceMode::FREECTRL:
          write_freectrl_cmd();
          write_IOs();
          break;
        case BaseHardwareFW01::InterfaceMode::INVALID:  default:
          // Don't write anything    
          return hardware_interface::return_type::ERROR;
          break;
      }

      last_write_time_ = time;
    }
    return hardware_interface::return_type::OK;
  }

  void BaseHardwareFW01::set_hw_read_freq (double freqrate) {
    if (freqrate <= 0) {
      RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "ERROR: Input for hw_read_freq is <= 0"); 
      return;
    }
    hw_read_freq_ = freqrate;
    double period = 1/freqrate;
    int32_t sec = (int32_t) std::floor(period);
    uint32_t nsec = (uint32_t) std::floor(1000000000*(period-sec));
    //hw_read_period_ = rclcpp::Duration(sec, nsec);
    hw_read_period_ = std::make_unique<rclcpp::Duration>(sec, nsec);
  }

  void BaseHardwareFW01::set_hw_write_freq (double freqrate) {
    if (freqrate == 0) {
      RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "ERROR: Input for hw_write_freq is <= 0"); 
      return;
    }
    hw_write_freq_ = freqrate;
    double period = 1/freqrate;
    int32_t sec = (int32_t) std::floor(period);
    uint32_t nsec = (uint32_t) std::floor(1000000000*(period-sec));
    //hw_write_period_ = rclcpp::Duration(sec, nsec);
    hw_write_period_ = std::make_unique<rclcpp::Duration>(sec, nsec);
  }
} // namespace fw01_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fw01_robot_driver::BaseHardwareFW01,hardware_interface::SystemInterface)







// https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html

// https://github.com/joshnewans/diffdrive_arduino/blob/main/src/diffdrive_arduino.cpp
// https://github.com/frankaemika/franka_ros2/blob/develop/franka_hardware/src/franka_hardware_interface.cpp
// https://github.com/NanyangBot/Four_Wheel_Steering_Mobile_Base/blob/nanyang_bot/swerve_drive_hardware/src/maxon_ros_interface.cpp
// https://github.com/husky/husky/blob/galactic-devel/husky_base/src/husky_hardware.cpp
// https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/src/hardware_interface.cpp
// https://github.com/ros-controls/ros2_control_demos





//https://github.com/ros-controls/ros2_control/tree/master/hardware_interface
//Cannot use duration/period parameter. This value basically period of controller manager, and same throughout all hardware interface
//Should add param so user can configure hwi's rate
// https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
//https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp
//https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/resource_manager.cpp
// 

//Check extension
// Check viisbility