#ifndef FW01_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define FW01_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "fw01_msgs/msg/battery_states.hpp"
#include "fw01_msgs/msg/io_states.hpp"
#include "fw01_msgs/srv/io_cmd.hpp"
#include "fw01_msgs/msg/steering_ctrl_fb.hpp"
#include "fw01_msgs/msg/robot_states.hpp"

namespace fw01_controllers
{

  class GPIOController : public controller_interface::ControllerInterface
  {
  public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_init() override;

  private:
    void setDefaultIO(void);
    bool setIOCmd(fw01_msgs::srv::IOCmd::Request::SharedPtr req, fw01_msgs::srv::IOCmd::Response::SharedPtr resp);
    void publishIOFb(void);
    void publishBattFb(void);
    void publishRobotStates(void);

    double battMinCapacity_;
    double battMaxCapacity_;

  protected:
    rclcpp::Service<fw01_msgs::srv::IOCmd>::SharedPtr set_io_srv_;

    std::shared_ptr<rclcpp::Publisher<fw01_msgs::msg::BatteryStates>> battery_pub_;
    std::shared_ptr<rclcpp::Publisher<fw01_msgs::msg::RobotStates>> robot_pub_;
    std::shared_ptr<rclcpp::Publisher<fw01_msgs::msg::IOStates>> io_pub_;

    fw01_msgs::msg::RobotStates robot_msg_;
    fw01_msgs::msg::BatteryStates battery_msg_;
    fw01_msgs::msg::IOStates io_msg_;

    enum CommandInterfaces
    {
      IO_CMD_LAMP_CTRL = 0u,
      IO_CMD_UNLOCK = 1,
      IO_CMD_LOWER_BEAM_HEADLAMP = 2,
      IO_CMD_LEFT_TURN_LAMP = 3,
      IO_CMD_RIGHT_TURN_LAMP = 4,
      IO_CMD_BRAKING_LAMP = 5,
    };

    enum StateInterfaces
    {
      IO_FB_LAMP_CTRL = 16,
      IO_FB_UNLOCK = 17,
      IO_FB_LOWER_BEAM_HEADLAMP = 18,
      IO_FB_LEFT_TURN_LAMP = 19,
      IO_FB_RIGHT_TURN_LAMP = 20,
      IO_FB_BRAKING_LAMP = 21,
      STEERING_CTRL_FB_GEAR = 22,
      STEERING_CTRL_FB_VELOCITY = 23,
      STEERING_CTRL_FB_STEERING = 24,
      STEERING_CTRL_FB_SLIPANGLE = 25,
      BMS_FB_VOLTAGE = 26,
      BMS_FB_CURRENT = 27,
      BMS_FB_REMAINING_CAPACITY = 28,
      INITIALIZED_FLAG = 29,
      // TODO: Add joint states here
    };  

  }; // GPIOController
}  // namespace fw01_controllers

#endif  // FW01_CONTROLLERS__GPIO_CONTROLLER_HPP_
