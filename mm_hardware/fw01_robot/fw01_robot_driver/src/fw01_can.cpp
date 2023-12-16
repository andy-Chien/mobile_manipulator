#include "fw01_robot_driver/fw01_can.hpp"

#include <fcntl.h>
#include <dirent.h>
//#include <linux/input.h>
#include <sys/stat.h>
//#include <pthread.h>
#include <time.h>

//#include <boost/bind.hpp>
//#include <boost/thread.hpp>

#include <mutex>
#include <rclcpp/logging.hpp>



namespace fw01_robot_driver {

  //YhsCanControl::YhsCanControl() : logger_(rclcpp::get_logger("YhsCanControl")) {
	//	logger_ = rclcpp::get_logger("YhsCanControl");
	//	connected_ = openConnection("can0");
	//}

	//YhsCanControl::YhsCanControl(const std::string& can_port, rclcpp::Logger& logger) : logger_(logger) {
	//YhsCanControl::YhsCanControl(const std::string& can_port) {
	YhsCanControl::YhsCanControl(const std::string& can_port, std::shared_ptr<rclcpp::Logger> logger) {
		//logger_ = get_logger(); //rclcpp::get_logger("YhsCanControl");
		logger_ = logger;
		memset(sendData_u_io_,0,8);
		initialized_ = false;
		connected_ = openConnection(can_port);

		//tau_command_.fill(0.);
		//franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
		//if (not franka::hasRealtimeKernel()) {
		//	rt_config = franka::RealtimeConfig::kIgnore;
		//	RCLCPP_WARN( logger,"You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
		//}
		//robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
	}

	bool YhsCanControl::openConnection(const std::string& can_port) {
		can_port_ = can_port;

		dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (dev_handler_ < 0) {
			RCLCPP_FATAL(*logger_, "ERROR: Cannot open CAN device!");
			return false;
		} else {
			RCLCPP_INFO(*logger_, "Succesfully open CAN device ...[OK]");
		}

		struct ifreq ifr;
		std::string can_name(can_port_); 
		strcpy(ifr.ifr_name,can_name.c_str());
		ioctl(dev_handler_,SIOCGIFINDEX, &ifr);
		fcntl(dev_handler_, F_SETFL, O_NONBLOCK);

		// bind socket to network interface
		struct sockaddr_can addr;
		memset(&addr, 0, sizeof(addr));
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
		if (ret < 0) {
			RCLCPP_FATAL(*logger_, ">>bind dev_handler error!\r\n");
			return false;
		}
		return true;
	}

	void YhsCanControl::closeConnection(void) {
		if (connected_) {
			close(dev_handler_);
			connected_ = false;
			initialized_ = false;
			dev_handler_ = 0;
			RCLCPP_INFO(*logger_, "Succesfully close CAN device ...[OK]");
		}
	}

	void YhsCanControl::writeJoints(uint8_t mode, int16_t vel_count_cmd, int16_t steer_angle_count_cmd, int16_t slip_angle_count_cmd) {
		std::lock_guard<std::mutex> lock(cmd_mutex_);

		//short linear = msg.steering_ctrl_cmd_velocity * 1000;
	  //short angular = msg.steering_ctrl_cmd_steering * 100;
	  //short slipangle = msg.steering_ctrl_cmd_slipangle * 100;
		int16_t linear = vel_count_cmd;
	  int16_t angular = steer_angle_count_cmd;
	  int16_t slipangle = slip_angle_count_cmd;
	  static unsigned char count_2 = 0;

	  unsigned char sendData_u_tem_[8] = {0};
  	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & mode);
  	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linear & 0x0f) << 4));
		sendData_u_tem_[1] = (linear >> 4) & 0xff;
	  sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linear >> 12));
  	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angular & 0x0f) << 4));
		sendData_u_tem_[3] = (angular >> 4) & 0xff;
		sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angular >> 12));
		sendData_u_tem_[4] = sendData_u_tem_[4] | (0xf0 & ((slipangle & 0x0f) << 4));
		sendData_u_tem_[5] = (slipangle >> 4) & 0xff;
		sendData_u_tem_[6] = sendData_u_tem_[6] | (0x0f & (slipangle >> 12));

		count_2 ++;
		if(count_2 == 16)	
			count_2 = 0;
		sendData_u_tem_[6] =  sendData_u_tem_[6] | count_2 << 4;
	
		sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

		send_frames_[0].can_id = 0x98C4D1D0; //Can only run on 0x98C4D1D0; 
    send_frames_[0].can_dlc = 8;

//RCLCPP_INFO(*logger_, "send vel %d, %d, %d, %d", mode, linear, angular, slipangle);
		memcpy(send_frames_[0].data, sendData_u_tem_, 8);

		int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
		{
      RCLCPP_WARN(*logger_, "send message failed, error code: %d",ret);
    }
	}
	
	void YhsCanControl::prepare_free_ctrl_cmd(unsigned char (&buf)[8], cmd_gear mode, int16_t left_value, int16_t right_value) {
		//short angularl = msg.free_ctrl_cmd_angle_lf * 100;
		//short angularr = msg.free_ctrl_cmd_angle_rf * 100;  
		//short linearl = msg.free_ctrl_cmd_velocity_lf * 1000;
		//short linearr = msg.free_ctrl_cmd_velocity_rf * 1000;

		memset(buf, 0, sizeof(buf));
		buf[0] = buf[0] | (0x0f & mode);
		buf[0] = buf[0] | (0xf0 & ((left_value & 0x0f) << 4));
		buf[1] = (left_value >> 4) & 0xff;
		buf[2] = buf[2] | (0x0f & (left_value >> 12));
		buf[2] = buf[2] | (0xf0 & ((right_value & 0x0f) << 4));
		buf[3] = (right_value >> 4) & 0xff;
		buf[4] = buf[4] | (0x0f & (right_value >> 12));
	}

	void YhsCanControl::writeJoints(const std::array<int16_t, 4>& swerve_cmd, const std::array<int16_t, 4>& wheel_cmd) {
		std::lock_guard<std::mutex> lock(cmd_mutex_);

    // Send in this following order: Front_Angle -> Rear_Angle -> Front_Velocity -> Rear_Velocity 
		send_frames_[0].can_id = 0x98C4D5D0; // Front_Angle
		send_frames_[1].can_id = 0x98C4D6D0; // Rear_Angle
		send_frames_[2].can_id = 0x98C4D3D0; // Front_Velocity
		send_frames_[3].can_id = 0x98C4D4D0; // Rear_Velocity

    // _cmd convention followed Quadrant I-IV, start from rf, lf, lr, rr
    int16_t right_values[4], left_values[4];
		right_values[0] = swerve_cmd[0]; 		left_values[0] = swerve_cmd[1]; //Front_Angle
		right_values[1] = swerve_cmd[3]; 		left_values[1] = swerve_cmd[2]; //Rear_Angle
		right_values[2] = wheel_cmd[0]; 		left_values[2] = wheel_cmd[1];  //Front_Velocity
		right_values[3] = wheel_cmd[3]; 		left_values[3] = wheel_cmd[2];  //Rear_Velocity

 //RCLCPP_INFO(*logger_, "Free Cmd Pos [%d, %d, %d, %d], [%d, %d, %d, %d]", swerve_cmd[0], swerve_cmd[1], swerve_cmd[2], swerve_cmd[3], wheel_cmd[0], wheel_cmd[1] , wheel_cmd[2] ,wheel_cmd[3]);
 
	  static unsigned char count_f[] = {0, 0, 0, 0};
	  unsigned char sendData_u_tem_[8] = {0};

    for (int j=0; j<4; j++) {
      prepare_free_ctrl_cmd(sendData_u_tem_, cmd_gear::freectrl, left_values[j], right_values[j]);
      count_f[j]++;
			if(count_f[j] >= 16)	
				count_f[j] = 0;
			sendData_u_tem_[6] = sendData_u_tem_[6] | count_f[j]  << 4;
			sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

    	send_frames_[j].can_dlc = 8;
			memcpy(send_frames_[j].data, sendData_u_tem_, 8);
		}

    int ret[4];
    for (int j=0; j<4; j++) {
			ret[j] = write(dev_handler_, &send_frames_[j], sizeof(send_frames_[j]));
		}

		for (int j=0; j<4; j++) {
			if (ret[j] <= 0) {
				RCLCPP_WARN(*logger_, "send message failed, error code: %d",ret[j]);
			} 
		}
	}

  void YhsCanControl::setIO(unsigned int index, bool value) {
		if (index >= 17) {
			return;
		}

		int bit_idx = index % 8;
    int buf_idx = ((index - bit_idx)/8);
		if (value) { // if set to true
			sendData_u_io_[buf_idx] |= (1 << bit_idx);
		} else { // if set to false
			sendData_u_io_[buf_idx] &= (0xFF - (1 << bit_idx));
		}
	} 

	void YhsCanControl::writeIOs(void) {
		std::lock_guard<std::mutex> lock(cmd_mutex_);

		static unsigned char count_1 = 0; // alive count.
		//memset(sendData_u_io_,0,8);

 		// data inside sendData_u_io[0-5] was populated from yhs_can_->setIO(id, val) function. 
		// no need to alter data here. just send whatever value set previously

	  count_1++;
	  if(count_1 == 16)	
		  count_1 = 0;
		sendData_u_io_[6] =  sendData_u_io_[6] | count_1 << 4;

		sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

		send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

		memcpy(send_frames_[0].data, sendData_u_io_, 8);

//RCLCPP_INFO(*logger_, "sendData_u_io [%x, %x, %x, %x, %x, %x, %x, %x]", send_frames_[0].data[0], send_frames_[0].data[1],send_frames_[0].data[2],send_frames_[0].data[3],
//	 send_frames_[0].data[4],send_frames_[0].data[5],send_frames_[0].data[6],send_frames_[0].data[7]);


		int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
		{
      RCLCPP_WARN(*logger_, "send message failed, error code: %d",ret);
    }
	}

  void YhsCanControl::extractWheelFb(unsigned char (&buf)[8], int16_t & wheel_vel, int32_t & wheel_enc ) {
		wheel_vel = (int16_t)(buf[1] << 8 | buf[0]);
		wheel_enc = (int32_t)(buf[5] << 24 | buf[4] << 16 | buf[3] << 8 | buf[2]);
	}

	void YhsCanControl::extractAngleFb(unsigned char (&buf)[8], int16_t & right_swerve_pos, int16_t & left_swerve_pos) {
		left_swerve_pos = (int16_t)(buf[1] << 8 | buf[0]);
		right_swerve_pos = (int16_t)(buf[3] << 8 | buf[2]);
	}

	void YhsCanControl::extractIOFb(unsigned char (&buf)[8], bool (&io_data)[43]) {
		for (int j=0; j<6; j++) {
			io_data[j*8+0] = (bool) (buf[j] & 0x01);
			io_data[j*8+1] = (bool) (buf[j] & 0x02);
			io_data[j*8+2] = (bool) (buf[j] & 0x04);
			if (j < 5) {
				io_data[j*8+3] = (bool) (buf[j] & 0x08);
				io_data[j*8+4] = (bool) (buf[j] & 0x10);
				io_data[j*8+5] = (bool) (buf[j] & 0x20);
				io_data[j*8+6] = (bool) (buf[j] & 0x40);
				io_data[j*8+7] = (bool) (buf[j] & 0x80);
			}
		}
	}

	void YhsCanControl::extractBMSFb(unsigned char (&buf)[8], uint16_t &volt_bits, int16_t &cur_bits, uint16_t &cap_bits) {
		volt_bits = (uint16_t) (buf[1] << 8 | buf[0]);
		cur_bits = (int16_t)(buf[3] << 8 | buf[2]);
		cap_bits = (uint16_t)(buf[5] << 8 | buf[4]);
	}

  void YhsCanControl::updateFlag (int index, bool value) {
		std::lock_guard<std::mutex> lock(flag_mutex_);
		if (index >= 9)
		  return;
		flag_updates_[index] = (value) ? 1 : 0;

		if (!initialized_) {
			int numUpdate = 0;
			for (int j=0; j<9; j++) {
				if (flag_updates_[j])
					numUpdate++;
			}
			if (numUpdate >= 9) {
				initialized_ = true;
			}
		}
	}

	void YhsCanControl::processUpdates(void) {
		int r = 0;
		while((r = read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0]))) >= 0) {
			crc_ = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];
			if(crc_ != recv_frames_[0].data[7])
			{
				RCLCPP_WARN(*logger_, "Received CAN frame with incorrect CRC. Message is discarded");
				continue;
			}

			//RCLCPP_INFO(*logger_, "Read %d, can_id %x", r, recv_frames_[0].can_id);

			switch (recv_frames_[0].can_id)
			{
				case 0x98C4D1EF: // ctrl_fb
				case 0x98C4D2EF: // steering_ctrl_fb. Treat both case the same
					ctrl_fb_gear_ = (uint8_t) 0x0f & recv_frames_[0].data[0];
					ctrl_fb_linear_ = (int16_t)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4);
					ctrl_fb_angular_ = (int16_t)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4);
					ctrl_fb_slipangle_ = (int16_t)((recv_frames_[0].data[6] & 0x0f) << 12 | recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] & 0xf0) >> 4);
					break;
				case 0x98C4D6EF: // lf_wheel_fb
					extractWheelFb(recv_frames_[0].data, wheel_velocity_count_[1], wheel_encoder_count_[1]); // Quadrant I-IV, start from rf, lf, lr, rr
					updateFlag(0,true);
					break;
				case 0x98C4D7EF: //lr_wheel_fb
					extractWheelFb(recv_frames_[0].data, wheel_velocity_count_[2], wheel_encoder_count_[2]);
					updateFlag(1,true);
					break;
				case 0x98C4D8EF: // rr_wheel_fb
					extractWheelFb(recv_frames_[0].data, wheel_velocity_count_[3], wheel_encoder_count_[3]);
					updateFlag(2,true);					
					break;
				case 0x98C4D9EF: //rf_wheel_fb
					extractWheelFb(recv_frames_[0].data, wheel_velocity_count_[0], wheel_encoder_count_[0]);
					updateFlag(3,true);
					break;
				case 0x98C4DCEF: //front_angle_fb
					extractAngleFb(recv_frames_[0].data, swerve_position_count_[0], swerve_position_count_[1]); // rf, lf
					updateFlag(4,true);
					break;
				case 0x98C4DDEF: //rear_angle_fb
					extractAngleFb(recv_frames_[0].data, swerve_position_count_[3], swerve_position_count_[2]); // rr, lr
					updateFlag(5,true);
					break;
				case 0x98C4DAEF: //io_fb
					extractIOFb(recv_frames_[0].data, io_fb_);
					updateFlag(6,true);
					break;
				case 0x98C4E1EF: //bms_fb
					extractBMSFb(recv_frames_[0].data, bms_fb_voltage_, bms_fb_current_, bms_fb_remaining_capacity_);
					updateFlag(7,true);
					break;
				case 0x98C4E2EF: //bms_flag_fb
					memcpy(bms_flag_fb_data_, recv_frames_[0].data, 8); 
					updateFlag(8,true);
					break;
				case 0x98C4EAEF: //error_fb // notused
					//memcpy(bms_flag_fb_data_, recv_frames_[0].data, 8); 
					break;
				default:
				  // comment out below warning after testing
					RCLCPP_WARN(*logger_, "FATAL ERROR: Received CAN message with unknown ID %x", recv_frames_[0].can_id);
					break;
			}

		} // end of while
	}

	void YhsCanControl::readJoints(int16_t (&swerve_pos_count)[4], int16_t (&wheel_vel_count)[4], int32_t (&wheel_enc_count)[4]) {
		for (int j=0; j<4; j++) {
			swerve_pos_count[j] = swerve_position_count_[j];
			wheel_vel_count[j]  = wheel_velocity_count_[j];
			wheel_enc_count[j]  = wheel_encoder_count_[j];
		}
	}

	bool YhsCanControl::readIO(unsigned int index, bool &buf) { // return false for invalid index
		if (index >=43) {
			return false;
		}
		buf = io_fb_[index];
		return true;
	}
	
	void YhsCanControl::readBMS(uint16_t &read_volt, int16_t &read_cur, uint16_t &read_cap) {
		read_volt = bms_fb_voltage_;
		read_cur = bms_fb_current_;
		read_cap = bms_fb_remaining_capacity_;
	}

  YhsCanControl::~YhsCanControl() {
		closeConnection();
	}
}
