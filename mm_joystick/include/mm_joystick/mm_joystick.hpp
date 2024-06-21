#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "mm_msgs/msg/servo_control.hpp"

class MMJoystick : public rclcpp::Node
{
public:
	MMJoystick(std::string node_name);
private:
	void initParameters();
	void readParameters();
	bool armEnable(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		return msg->axes[arm_en_axes] < en_threshold;
	};
	bool baseEnable(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		return msg->axes[base_en_axes] < en_threshold;
	};
	bool hybridEnable(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		return msg->buttons[hybrid_en_button];
	};
	bool singleEnable(const sensor_msgs::msg::Joy::SharedPtr msg)
	{
		for(auto & [joint, button] : joint_button)
		{
			if(msg->buttons[button])
			{
				single_joint = joint;
				return true;
			}
		}
		return false;
	};

	void getArmCmd(geometry_msgs::msg::Twist& cmd);
	void getBaseCmd(geometry_msgs::msg::Twist& cmd);
	void getHybridCmd(geometry_msgs::msg::Twist& arm_cmd, geometry_msgs::msg::Twist& base_cmd);
	void checkCmdFrameSwitch(const sensor_msgs::msg::Joy::SharedPtr msg);
	void joysticMsgCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

	float dead_zone, en_threshold, base_trans_limit, base_rotat_limit, arm_trans_limit, arm_rotat_limit;
    uint8_t x_axes, y_axes, z_axes, rx_axes, ry_axes, rz_axes, base_en_axes, arm_en_axes;
	uint8_t hybrid_en_button, arm_frame_switch_button, base_frame_switch_button;
	bool frame_switched = false;
	bool lock_control_mode = false;

	std::string node_name_;
	std::string single_joint;
	std::vector<float> joy_val;
	std::map<std::string, uint8_t> joint_button;
	float scale = 1 /  (1 - dead_zone);
	uint8_t locked_control_mode = mm_msgs::msg::ServoControl::STOP;
	uint8_t last_control_mode = mm_msgs::msg::ServoControl::STOP;
	uint8_t arm_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
	uint8_t base_cmd_frame = mm_msgs::msg::ServoControl::BASE_FRAME;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
	rclcpp::Publisher<mm_msgs::msg::ServoControl>::SharedPtr cmd_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub;
};
