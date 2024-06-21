#include "mm_joystick/mm_joystick.hpp"

MMJoystick::MMJoystick(std::string node_name)
: Node(node_name)
{
	initParameters();
	readParameters();
	node_name_ = node_name;
	joy_val.resize(6);
	joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(
				&MMJoystick::joysticMsgCallback, this, std::placeholders::_1));
	cmd_pub = this->create_publisher<mm_msgs::msg::ServoControl>("/mm_servo_controller/cmd_vel", 1);
	twist_cmd_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mb_servo_controller/cmd_vel", 1);
}

void MMJoystick::initParameters()
{
	this->declare_parameter<uint8_t>("x_axes", 255);
	this->declare_parameter<uint8_t>("y_axes", 255);
	this->declare_parameter<uint8_t>("z_axes", 255);
	this->declare_parameter<uint8_t>("rx_axes", 255);
	this->declare_parameter<uint8_t>("ry_axes", 255);
	this->declare_parameter<uint8_t>("rz_axes", 255);
	this->declare_parameter<uint8_t>("arm_en_axes", 255);
	this->declare_parameter<uint8_t>("base_en_axes", 255);
	this->declare_parameter<uint8_t>("hybrid_en_button", 255);
	this->declare_parameter<uint8_t>("arm_frame_switch_button", 255);
	this->declare_parameter<uint8_t>("base_frame_switch_button", 255);
	this->declare_parameter<float>("dead_zone", NAN);
	this->declare_parameter<float>("en_threshold", NAN);
	this->declare_parameter<float>("arm_trans_limit", NAN);
	this->declare_parameter<float>("arm_rotat_limit", NAN);
	this->declare_parameter<float>("base_trans_limit", NAN);
	this->declare_parameter<float>("base_rotat_limit", NAN);
	this->declare_parameter<std::vector<int>>("single_joint.buttons", std::vector<int>());
	this->declare_parameter<std::vector<std::string>>("single_joint.joints", std::vector<std::string>());
}

void MMJoystick::readParameters()
{
	x_axes = this->get_parameter("x_axes").as_int();
	y_axes = this->get_parameter("y_axes").as_int();
	z_axes = this->get_parameter("z_axes").as_int();
	rx_axes = this->get_parameter("rx_axes").as_int();
	ry_axes = this->get_parameter("ry_axes").as_int();
	rz_axes = this->get_parameter("rz_axes").as_int();
	arm_en_axes = this->get_parameter("arm_en_axes").as_int();
	base_en_axes = this->get_parameter("base_en_axes").as_int();
	hybrid_en_button = this->get_parameter("hybrid_en_button").as_int();
	arm_frame_switch_button = this->get_parameter("arm_frame_switch_button").as_int();
	base_frame_switch_button = this->get_parameter("base_frame_switch_button").as_int();
	dead_zone = this->get_parameter("dead_zone").as_double();
	en_threshold = this->get_parameter("en_threshold").as_double();
	arm_trans_limit = this->get_parameter("arm_trans_limit").as_double();
	arm_rotat_limit = this->get_parameter("arm_rotat_limit").as_double();
	base_trans_limit = this->get_parameter("base_trans_limit").as_double();
	base_rotat_limit = this->get_parameter("base_rotat_limit").as_double();
	auto joints = this->get_parameter("single_joint.joints").as_string_array();
	auto buttons = this->get_parameter("single_joint.buttons").as_integer_array();
	if(joints.size() != buttons.size())
	{
		RCLCPP_WARN(this->get_logger(), 
			"size of joints and buttons are different! single joint control doesn't enable");
		return;
	}

	for(unsigned i = 0; i < joints.size(); i++)
	{
		joint_button[joints[i]] = buttons[i];
	}
}

void MMJoystick::getArmCmd(geometry_msgs::msg::Twist& cmd)
{
	cmd.linear.x = joy_val[0] * scale * arm_trans_limit;
	cmd.linear.y = joy_val[1] * scale * arm_trans_limit;
	cmd.linear.z = joy_val[2] * scale * arm_trans_limit;
	cmd.angular.x = joy_val[3] * scale * arm_rotat_limit;
	cmd.angular.y = joy_val[4] * scale * arm_rotat_limit;
	cmd.angular.z = joy_val[5] * scale * arm_rotat_limit;
}

void MMJoystick::getBaseCmd(geometry_msgs::msg::Twist& cmd)
{
	cmd.linear.x = joy_val[0] * scale * base_trans_limit;
	cmd.linear.y = joy_val[1] * scale * base_trans_limit;
	cmd.angular.z = joy_val[5] * scale * base_rotat_limit;
}

void MMJoystick::getHybridCmd(geometry_msgs::msg::Twist& arm_cmd, geometry_msgs::msg::Twist& base_cmd)
{
	arm_cmd.linear.x = joy_val[0] * scale * arm_trans_limit;
	arm_cmd.linear.y = joy_val[1] * scale * arm_trans_limit;
	arm_cmd.linear.z = joy_val[4] * scale * arm_trans_limit;
	base_cmd.linear.x = joy_val[2] * scale * base_trans_limit;
	base_cmd.linear.y = joy_val[5] * scale * base_trans_limit;
	base_cmd.angular.z = joy_val[3] * scale * base_rotat_limit;
}

void MMJoystick::checkCmdFrameSwitch(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	if(msg->buttons[arm_frame_switch_button] && !frame_switched)
	{
		frame_switched = true;
		switch (arm_cmd_frame)
		{
		case mm_msgs::msg::ServoControl::BASE_FRAME:
			arm_cmd_frame = mm_msgs::msg::ServoControl::TOOL_FRAME;
			RCLCPP_INFO(this->get_logger(), "Arm cmd frame switch to TOOL_FRAME");
			break;
		case mm_msgs::msg::ServoControl::TOOL_FRAME:
			arm_cmd_frame = mm_msgs::msg::ServoControl::WORLD_FRAME;
			RCLCPP_INFO(this->get_logger(), "Arm cmd frame switch to WORLD_FRAME");
			break;
		case mm_msgs::msg::ServoControl::WORLD_FRAME:
			arm_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
			RCLCPP_INFO(this->get_logger(), "Arm cmd frame switch to BASE_FRAME");
			break;
		default:
			arm_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
			RCLCPP_INFO(this->get_logger(), "Arm cmd frame switch to BASE_FRAME");
			break;
		}
	}

	if(msg->buttons[base_frame_switch_button] && !frame_switched)
	{
		frame_switched = true;
		switch (base_cmd_frame)
		{
		case mm_msgs::msg::ServoControl::BASE_FRAME:
			base_cmd_frame = mm_msgs::msg::ServoControl::WORLD_FRAME;
			RCLCPP_INFO(this->get_logger(), "Base cmd frame switch to WORLD_FRAME");
			break;
		case mm_msgs::msg::ServoControl::WORLD_FRAME:
			base_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
			RCLCPP_INFO(this->get_logger(), "Base cmd frame switch to BASE_FRAME");
			break;
		default:
			base_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
			RCLCPP_INFO(this->get_logger(), "Base cmd frame switch to BASE_FRAME");
			break;
		}
	}

	if(!msg->buttons[arm_frame_switch_button] && 
		!msg->buttons[base_frame_switch_button])
	{
		frame_switched = false;
	}
}

void MMJoystick::joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	bool arm_enable = armEnable(msg);
	bool base_enable = baseEnable(msg);
	bool hybrid_enable = hybridEnable(msg);
	bool single_enable = singleEnable(msg);

	mm_msgs::msg::ServoControl cmd;
	if(!arm_enable && !base_enable){
		cmd.control_mode = mm_msgs::msg::ServoControl::STOP;
		lock_control_mode = false;
	}
	else if(lock_control_mode)
	{
		switch (locked_control_mode)
		{
		case mm_msgs::msg::ServoControl::HYBRID:
			if(hybrid_enable && arm_enable && base_enable){
				cmd.control_mode = mm_msgs::msg::ServoControl::HYBRID;
			}else{
				cmd.control_mode = mm_msgs::msg::ServoControl::STOP;
			}
			break;
		case mm_msgs::msg::ServoControl::SINGLE_JOINT:
			if(single_enable && arm_enable){
				cmd.control_mode = mm_msgs::msg::ServoControl::SINGLE_JOINT;
			}else{
				cmd.control_mode = mm_msgs::msg::ServoControl::STOP;
			}
			break;
		default:
			cmd.control_mode = mm_msgs::msg::ServoControl::STOP;
			break;
		}
	}
	else if(arm_enable && base_enable)
	{
		if(hybrid_enable){
			lock_control_mode = true;
			cmd.control_mode = mm_msgs::msg::ServoControl::HYBRID;
			locked_control_mode = mm_msgs::msg::ServoControl::HYBRID;
		}else{
			cmd.control_mode = mm_msgs::msg::ServoControl::COMBINE;
		}
	}
	else if(arm_enable)
	{
		if(single_enable){
			lock_control_mode = true;
			cmd.control_mode = mm_msgs::msg::ServoControl::SINGLE_JOINT;
			locked_control_mode = mm_msgs::msg::ServoControl::SINGLE_JOINT;
		}else{
			cmd.control_mode = mm_msgs::msg::ServoControl::ARM;
		}
	}
	else if(base_enable){
		cmd.control_mode = mm_msgs::msg::ServoControl::BASE;
	}else{
		return;
	}

	if(cmd.control_mode != mm_msgs::msg::ServoControl::STOP)
	{
		joy_val[0] = (msg->axes[x_axes] > dead_zone) ? msg->axes[x_axes] - dead_zone : 
					(msg->axes[x_axes] < -1 * dead_zone) ? msg->axes[x_axes] + dead_zone : 0;
		joy_val[1] = (msg->axes[y_axes] > dead_zone) ? msg->axes[y_axes] - dead_zone : 
					(msg->axes[y_axes] < -1 * dead_zone) ? msg->axes[y_axes] + dead_zone : 0;
		joy_val[2] = (msg->axes[z_axes] > dead_zone) ? msg->axes[z_axes] - dead_zone : 
					(msg->axes[z_axes] < -1 * dead_zone) ? msg->axes[z_axes] + dead_zone : 0;
		joy_val[3] = (msg->axes[rx_axes] > dead_zone) ? msg->axes[rx_axes] - dead_zone : 
					(msg->axes[rx_axes] < -1 * dead_zone) ? msg->axes[rx_axes] + dead_zone : 0;
		joy_val[4] = (msg->axes[ry_axes] > dead_zone) ? msg->axes[ry_axes] - dead_zone : 
					(msg->axes[ry_axes] < -1 * dead_zone) ? msg->axes[ry_axes] + dead_zone : 0;
		joy_val[5] = (msg->axes[rz_axes] > dead_zone) ? msg->axes[rz_axes] - dead_zone : 
					(msg->axes[rz_axes] < -1 * dead_zone) ? msg->axes[rz_axes] + dead_zone : 0;
	}

	switch (cmd.control_mode)
	{
	case mm_msgs::msg::ServoControl::HYBRID:
		getHybridCmd(cmd.arm_cmd, cmd.base_cmd);
		break;
	case mm_msgs::msg::ServoControl::COMBINE:
		getArmCmd(cmd.arm_cmd);
		break;
	case mm_msgs::msg::ServoControl::SINGLE_JOINT:
		cmd.joint = single_joint;
		cmd.value = joy_val[0] * scale * arm_rotat_limit;
		break;
	case mm_msgs::msg::ServoControl::ARM:
		getArmCmd(cmd.arm_cmd);
		break;
	case mm_msgs::msg::ServoControl::BASE:
		getBaseCmd(cmd.base_cmd);
		break;
	default:
		break;
	}

	if(!cmd.control_mode == mm_msgs::msg::ServoControl::STOP ||
		!last_control_mode == mm_msgs::msg::ServoControl::STOP)
	{
		last_control_mode = cmd.control_mode;
		cmd.header.stamp = this->get_clock()->now();
		cmd.header.frame_id = node_name_;
		cmd.arm_cmd_frame = arm_cmd_frame;
		cmd.base_cmd_frame = base_cmd_frame;
		geometry_msgs::msg::TwistStamped twist_cmd;
		twist_cmd.header = cmd.header;
		twist_cmd.twist = cmd.base_cmd;
		cmd_pub->publish(cmd);
		twist_cmd_pub->publish(twist_cmd);
	}
	else{
		checkCmdFrameSwitch(msg);
	}
}