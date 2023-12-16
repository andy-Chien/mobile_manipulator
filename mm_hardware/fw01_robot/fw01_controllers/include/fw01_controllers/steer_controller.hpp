#ifndef FW01_CONTROLLERS__STEER_CONTROLLER_HPP_
#define FW01_CONTROLLERS__STEER_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "fw01_msgs/srv/steer_cmd.hpp"
#include "fw01_msgs/srv/transverse_cmd.hpp"
#include "fw01_msgs/srv/steering_ctrl_cmd.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace fw01_controllers
{
  class SteerController : public controller_interface::ControllerInterface
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
    bool setSteerCmd(fw01_msgs::srv::SteerCmd::Request::SharedPtr req, fw01_msgs::srv::SteerCmd::Response::SharedPtr resp);
    bool setTransverseCmd(fw01_msgs::srv::TransverseCmd::Request::SharedPtr req, fw01_msgs::srv::TransverseCmd::Response::SharedPtr resp);
    bool setSteeringCtrlCmd(fw01_msgs::srv::SteeringCtrlCmd::Request::SharedPtr req, fw01_msgs::srv::SteeringCtrlCmd::Response::SharedPtr resp);
    bool stopRobotTrigger(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);
    
    void msgTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    //void publishSteeringCtrlFb(void);


  protected:
    rclcpp::Service<fw01_msgs::srv::SteerCmd>::SharedPtr set_steer_srv_;
    rclcpp::Service<fw01_msgs::srv::TransverseCmd>::SharedPtr set_transverse_srv_;
    rclcpp::Service<fw01_msgs::srv::SteeringCtrlCmd>::SharedPtr set_steering_ctrl_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_robot_srv_;

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmdvel_sub_;

    //std::shared_ptr<rclcpp::Publisher<fw01_msgs::msg::SteeringCtrlFb>> steering_ctrl_fb_pub_;
    //fw01_msgs::msg::SteeringCtrlFb steering_ctrl_fb_msg_;
   
    enum CommandInterfaces
    {
      STEERING_CTRL_CMD_GEAR = 0u,
      STEERING_CTRL_CMD_VELOCITY = 1,
      STEERING_CTRL_CMD_STEERING = 2,
      STEERING_CTRL_CMD_SLIPANGLE = 3,
    };

    enum StateInterfaces
    {
       INITIALIZED_FLAG = 0u,
    };  

    enum cmd_gear : std::uint8_t 
    {
      disable = 0,
      park = 1,
      neutral = 2,
      //FR-gear = 4 // not used
      steer = 6, //4T4D
      transverse = 7,
      freectrl = 8
    };

  }; // SteerController
}  // namespace fw01_controllers

#endif  // FW01_CONTROLLERS__STEER_CONTROLLER_HPP_
