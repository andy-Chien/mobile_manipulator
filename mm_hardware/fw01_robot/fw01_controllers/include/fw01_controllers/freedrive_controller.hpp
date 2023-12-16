#ifndef FW01_CONTROLLERS__FREEDRIVE_CONTROLLER_HPP_
#define FW01_CONTROLLERS__FREEDRIVE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "fw01_msgs/msg/freedrive.hpp"
#include "fw01_msgs/msg/freedrive_joint.hpp"


namespace fw01_controllers
{
  class FreedriveController : public controller_interface::ControllerInterface
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
    //bool setFreedriveCmd(fw01_msgs::srv::FreedriveCmd::Request::SharedPtr req, fw01_msgs::srv::FreedriveCmd::Response::SharedPtr resp);
    //bool setFreedriveIndividualCmd(fw01_msgs::srv::FreedriveIndividualCmd::Request::SharedPtr req, fw01_msgs::srv::FreedriveIndividualCmd::Response::SharedPtr resp);
    bool stopRobotTrigger(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp);
    void stopRobot(void);

    void setCombinedJointCallback(const fw01_msgs::msg::Freedrive::SharedPtr msg);
    void setIndividualJointCallback(const fw01_msgs::msg::FreedriveJoint::SharedPtr msg);

  protected:
    //rclcpp::Service<fw01_msgs::srv::FreedriveCmd>::SharedPtr set_freedrive_srv_;
    //rclcpp::Service<fw01_msgs::srv::FreedriveIndividualCmd>::SharedPtr set_freedrive_individual_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_robot_srv_;
   
    std::shared_ptr<rclcpp::Subscription<fw01_msgs::msg::Freedrive>> freedrive_all_sub_;
    std::shared_ptr<rclcpp::Subscription<fw01_msgs::msg::FreedriveJoint>> freedrive_joint_sub_;

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

  }; // FreedriveController
}  // namespace fw01_controllers

#endif  // FW01_CONTROLLERS__FREEDRIVE_CONTROLLER_HPP_
