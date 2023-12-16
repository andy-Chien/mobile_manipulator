#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "fw01_msgs/msg/freedrive.hpp"
#include "fw01_msgs/msg/freedrive_joint.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MoveJoy : public rclcpp::Node
{
  public:
    MoveJoy()
    : Node("move_joy_node")
    {
      velx_ = 0;
      vely_ = 0;
      velth_ = 0;

      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("mobile_standard_driver_controller/cmd_vel", 10);
      freedrive_robot_pub_ = this->create_publisher<fw01_msgs::msg::Freedrive>("mobile_freedrive_driver_controller/freedrive_robot_cmd", 10);
      freedrive_joint_pub_ = this->create_publisher<fw01_msgs::msg::FreedriveJoint>("mobile_freedrive_driver_controller/freedrive_joint_cmd", 10);

      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MoveJoy::joy_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(100ms, std::bind(&MoveJoy::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = velx_;
      twist_msg.linear.y = vely_;
      twist_msg.angular.z = velth_;

      twist_pub_->publish(twist_msg);

      for (int j=0; j<4; j++) {
        if (has_joint_cmd_[j]) {
          auto joint_cmd = fw01_msgs::msg::FreedriveJoint();
          joint_cmd.quad_id = j;
          joint_cmd.swerve_pos_rad = swerve[j];
          joint_cmd.wheel_vel_rad = wheel[j];
          freedrive_joint_pub_->publish(joint_cmd);
          has_joint_cmd_[j] = false;
        }
      }

      if (has_robot_cmd_) {
        auto robot_cmd = fw01_msgs::msg::Freedrive();
        robot_cmd.rf_swerve_pos_rad = swerve[0];
        robot_cmd.lf_swerve_pos_rad = swerve[1];
        robot_cmd.lr_swerve_pos_rad = swerve[2];
        robot_cmd.rr_swerve_pos_rad = swerve[3];
        robot_cmd.rf_wheel_vel_rad = wheel[0];
        robot_cmd.lf_wheel_vel_rad = wheel[1];
        robot_cmd.lr_wheel_vel_rad = wheel[2];
        robot_cmd.rr_wheel_vel_rad = wheel[3];
        freedrive_robot_pub_->publish(robot_cmd);
        has_robot_cmd_ = false;
      }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
    { 
      // axes 0 (side),1 () 3
      if ((msg->axes[2]) < 0.5) {
        velx_ = msg->axes[1] * 0.25;
        vely_ = msg->axes[0] * 0.25;
        velth_ = msg->axes[3] * M_PI / 3;  //scale down
      }
      else {
        velx_ = 0;
        vely_ = 0;
        velth_ = 0;  
      }

      double vel_rad = hypotf64(velx_, vely_) / 0.12; // convert m/s to rad
      double slipangle_rad = atan2(vely_, velx_); //unit in degree. atan2 (dy, dx)
      if ((slipangle_rad < (-M_PI/2)) || (slipangle_rad > (M_PI/2))) {
        vel_rad *= -1;
        slipangle_rad += (slipangle_rad < -M_PI/2) ? M_PI : - M_PI; 
      } 

      if (((msg->axes[2]) < 0.5)) {
        // 1, 3, 2, 0
        if (msg->buttons[1] == 1) {
          swerve[0] = slipangle_rad;
          wheel[0]= vel_rad;
          has_joint_cmd_[0] = true;
        }
        if  (msg->buttons[3] == 1) {
          swerve[1] = slipangle_rad;
          wheel[1]= vel_rad;
          has_joint_cmd_[1] = true;
          }
        if  (msg->buttons[2] == 1) {
          swerve[2] = slipangle_rad;
          wheel[2]= vel_rad;
          has_joint_cmd_[2] = true;
        }
        if (msg->buttons[0] == 1) {
          swerve[3] = slipangle_rad;
          wheel[3]= vel_rad;
          has_joint_cmd_[3] = true;
        }
        if ((msg->buttons[0] != 1) && (msg->buttons[1] != 1) && (msg->buttons[2] != 1) && (msg->buttons[3] != 1)) {
          for (int j=0; j < 4; j++) {
            swerve[j] = slipangle_rad;
            wheel[j]= vel_rad;
          }
          has_robot_cmd_ = true;
        }
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<fw01_msgs::msg::Freedrive>::SharedPtr freedrive_robot_pub_;
    rclcpp::Publisher<fw01_msgs::msg::FreedriveJoint>::SharedPtr freedrive_joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    double velx_;
    double vely_;
    double velth_;
    bool has_joint_cmd_[4];
    bool has_robot_cmd_;
    double swerve[4];
    double wheel[4];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveJoy>());
  rclcpp::shutdown();
  return 0;
}