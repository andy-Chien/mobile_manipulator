#include <rclcpp/rclcpp.hpp>
#include "mm_joystick/mm_joystick.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	std::string node_name("mm_joystick");
	rclcpp::spin(std::make_shared<MMJoystick>(node_name));
	rclcpp::shutdown();
	return 0;
}