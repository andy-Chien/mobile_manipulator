#ifndef FW01_ROBOT_DRIVER__FW01_CAN_HPP_
#define FW01_ROBOT_DRIVER__FW01_CAN_HPP_

#include <mutex>
#include <string>

#include <rclcpp/logger.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace fw01_robot_driver
{
	class YhsCanControl {
		enum cmd_gear : std::uint8_t 
		{
			disable = 0,
			park = 1,
			neutral = 2,
			//FR-gear = 4 // not used
			normal = 6, //4T4D
			transverse = 7,
			freectrl = 8
		};

    public:
    	//YhsCanControl();
			YhsCanControl(const std::string& can_port, std::shared_ptr<rclcpp::Logger> logger);
	    ~YhsCanControl();

      void writeJoints(uint8_t mode, int16_t vel_count_cmd, int16_t steer_angle_count_cmd, int16_t slip_angle_count_cmd);
			void writeJoints(const std::array<int16_t, 4>& swerve_cmd, const std::array<int16_t, 4>& wheel_cmd);
		  void setIO(unsigned int index, bool value);
			void writeIOs(void);

			void processUpdates(void); // recv
			bool readIO(unsigned int index, bool &buf); // return false for invalid index
			void readBMS(uint16_t &read_volt, int16_t &read_cur, uint16_t &read_cap);
			void readJoints(int16_t (&swerve_pos_count)[4], int16_t (&wheel_vel_count)[4], int32_t (&wheel_enc_count)[4]);

		public:
			bool initialized_;

    private:
		  bool connected_;
			int dev_handler_;
			std::shared_ptr<rclcpp::Logger> logger_;
      std::string can_port_;

			// We probably don't need to store output command as internal variables
			//std::array<int16_t, 4> joint_swerve_command_; // for free_ctrl. [-32768:32767] unit 0.01deg/unit [-327.68deg to 327.67 deg] 
			//std::array<int16_t, 4> joint_wheel_command_; // for free ctrl. [-32768:32767] unit 0.001m/s/unit [-32.768 m/s to 32.767 m/s] 
			//uint8_t body_ctrl_gear_command_;
  		//int16_t body_velocity_command_; // [-32768-32767] unit is 0.001 m/s/bit
			//int16_t body_steer_angle_command_; // [-32768-32767] unit  is 0.01deg /bit
			//int16_t body_slip_angle_command_; // [-32768-32767] unit  is 0.01deg /bit
			//uint8_t alive_cnt_;

      // Store received message ctrl_fb, steer_ctrl_fb as internal variables
			uint8_t ctrl_fb_gear_;
  		int16_t ctrl_fb_linear_;    // [-32768-32767] unit is 0.001 m/s/bit
			int16_t ctrl_fb_angular_;   // [-32768-32767] unit  is 0.01deg /bit
			int16_t ctrl_fb_slipangle_; // [-32768-32767] unit  is 0.01deg /bit

			// Store IO
			bool io_fb_[43] = {false};
			uint16_t bms_fb_voltage_ = 0; // 0.01V /bit
			int16_t  bms_fb_current_ = 0; // 0.01A /bit
			uint16_t bms_fb_remaining_capacity_ = 0; // 0.01Ah /bit
			//unsigned char io_fb_data_[8] = {0}; // Old structure, not use and can delete this
			//unsigned char bms_fb_data_[8] = {0};
			unsigned char bms_flag_fb_data_[8] = {0}; // Extract this data if you want to use this	

			// Store Joint Data. 
			int16_t swerve_position_count_[4] = {0};
			int16_t wheel_velocity_count_[4] = {0};
			int32_t wheel_encoder_count_[4] = {0};		

      // Data structure from drivers
      std::mutex cmd_mutex_;
			std::mutex flag_mutex_;
	    unsigned char sendData_u_io_[8] = {0};
	    unsigned char sendData_u_vel_[8] = {0};
			int8_t flag_updates_[9] = {0};
	
			can_frame send_frames_[4]; // 
			can_frame recv_frames_[1]; // Actually does not need array, but we follow driver code for readability & compatibility
			unsigned char crc_;
	
    public: // change to private after testing
			bool openConnection(const std::string& can_port);
			void closeConnection(void); 
		  void prepare_free_ctrl_cmd(unsigned char (&buf)[8], cmd_gear mode, int16_t left_value, int16_t right_value);
		  void extractWheelFb(unsigned char (&buf)[8], int16_t & wheel_vel, int32_t & wheel_enc );
			void extractAngleFb(unsigned char (&buf)[8], int16_t & right_swerve_pos, int16_t & left_swerve_pos);
			void extractIOFb(unsigned char (&buf)[8], bool (&io_data)[43]);
			void extractBMSFb(unsigned char (&buf)[8], uint16_t &volt_bits, int16_t &cur_bits, uint16_t &cap_bits);
			void updateFlag (int index, bool value);
  }; // class YhsCanControl
} // namespace fw01_robot_driver

#endif // FW01_ROBOT_DRIVER__FW01_CAN_HPP_