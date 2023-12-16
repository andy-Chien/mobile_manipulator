#ifndef FW01_ROBOT_DRIVER__FW01_INTERFACE_HPP_
#define FW01_ROBOT_DRIVER__FW01_INTERFACE_HPP_

#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// #include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
// #include "rclcpp/macros.hpp"
// #include "rclcpp/time.hpp"

#include "fw01_robot_driver/fw01_can.hpp"

namespace fw01_robot_driver
{
	class BaseHardwareFW01 : public hardware_interface::SystemInterface
	{
	public:
		RCLCPP_SHARED_PTR_DEFINITIONS(BaseHardwareFW01)
		
		virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
		virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
		virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
		virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
		virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
		virtual hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
		virtual hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
		virtual hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
    virtual hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
		virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
		virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

		void set_hw_read_freq (double freqrate);
		void set_hw_write_freq (double freqrate);
	  void send_zeros_command (void);

	public:
		enum InterfaceMode : std::int8_t 
    {
			INVALID = -1,
      UNINITIALIZED = 0,
      STEERCTRL = 1,
      FREECTRL = 2
    };


  private:
		void write_IOs(void);
  	void write_steer_cmd(void);
  	void write_freectrl_cmd(void);
		void send_zeros_command (fw01_robot_driver::BaseHardwareFW01::InterfaceMode mode);
		BaseHardwareFW01::InterfaceMode verify_interface_mode (const std::vector<std::string> & interfaces);
  
	private:
    // TOTRY:
		//const int kSize_io_cmd = 17; // Don't use them all but io_cmd usable bit from 0-16. Hardcode
		//const int kSize_io_fb  = 43; // Don't use them all but io_fb usable bit from 0-42
		//const int kSize_bms_fb = 3;
		//std::array<double> actual_io_cmd_bits_copy_;  
	  //std::array<double> actual_io_fb_bits_copy_;  
		//std::array<double> actual_bms_fb_bits_copy_;  

		// angle 4 joint, speed 4 hubs, encoder 4 hubs. canname info, baudrate
		// Hardware Parameterserror: ‘rclcpp::Duration::Duration()’ is private within this context

		double hw_read_freq_;
		double hw_write_freq_;
		double hw_wheel_diam_;
		std::string can_port_;
		std::unique_ptr<fw01_robot_driver::YhsCanControl> yhs_can_;
	

    // IO Commands
		std::array<double, 17> actual_io_cmd_bits_copy_;  // Don't use them all but io_cmd usable bit from 0-16. Hardcode

    // IO BMS States
	  std::array<double, 43> actual_io_fb_bits_copy_;  // Don't use them all but io_fb usable bit from 0-42
		std::array<double, 3> actual_bms_fb_bits_copy_; // 

    // Swerve Commands and States
    std::vector<double> hw_swerve_commands_position_;
    std::vector<double> hw_swerve_states_position_;
		std::vector<double> hw_swerve_states_velocity_;

    // Wheel Commands and States
		std::vector<double> hw_wheel_commands_velocity_;
		std::vector<double> hw_wheel_states_position_;
		std::vector<double> hw_wheel_states_velocity_;
		std::vector<int32_t> hw_last_wheel_enc_count_;

		// GPIO steering_ctrl_cmd
		double hw_steering_ctrl_cmd_gear_; // gear, velocity, angular, slipangle
    double hw_steering_ctrl_cmd_velocity_;
    double hw_steering_ctrl_cmd_steering_;
    double hw_steering_ctrl_cmd_slipangle_;

		// GPIO steering_ctrl_fb
		double hw_steering_ctrl_fb_gear_; // gear, velocity, angular, slipangle
    double hw_steering_ctrl_fb_velocity_;
    double hw_steering_ctrl_fb_steering_;
    double hw_steering_ctrl_fb_slipangle_;
		double hw_system_interface_initialized_;

	
    //bool free_ctrl_controller_running_;
		//bool steer_ctrl_controller_running_; // Using default protocol of YHS "Gear Mode". Mode 6 for 4T4D, Mode 7 for transverse 
		BaseHardwareFW01::InterfaceMode hw_interface_mode_;

	  bool first_read_pass_;
		bool first_write_pass_;
		rclcpp::Time last_read_time_;
		rclcpp::Time last_write_time_;
		std::unique_ptr<rclcpp::Duration> hw_read_period_;
		std::unique_ptr<rclcpp::Duration> hw_write_period_;
		//rclcpp::Duration hw_read_period_;
		//rclcpp::Duration hw_write_period_;

	
	  enum side : std::uint8_t // Follow Quadrant I-IV, start from rf, lf, lr, rr
    {
      rf = 0,
      lf = 1,
      lr = 2,
      rr = 3
    };
	};
}
#endif // FW01_ROBOT_DRIVER__FW01_INTERFACE_HPP_
