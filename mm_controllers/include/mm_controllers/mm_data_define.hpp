
#ifndef MM_CONTROLLERS__MM_DATA_DEFINE_HPP_
#define MM_CONTROLLERS__MM_DATA_DEFINE_HPP_

#include <map>
#include <vector>
#include <string>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

namespace mm_controllers
{
namespace data_define
{
// TODO(andy): Abandon double[], use std::vector or Eigen::Vector
typedef double Double2[2];

constexpr auto DEFAULT_HALTED_THRESHOLD = 0.01;
constexpr auto DEFAULT_TOLERANCE = 0.003;
constexpr auto DEFAULT_SMALL_ENOUGH_FLOAT = 0.0001;
constexpr auto DEFAULT_MAX_ICR_LENGTH = 1000; // 1 km
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";

struct JointLimits
{
public:
  bool has_pos_limit{false};
  bool has_vel_limit{false};
  bool has_acc_limit{false};
  bool has_jerk_limit{false};
  double pos_max{0};
  double pos_min{0};
  double vel_max{0};
  double vel_min{0};
  double acc_max{0};
  double acc_min{0};
  double jerk_max{0};
  double jerk_min{0};
  double vel{0};
  double acc{0};
  double jerk{0};
};

struct InterfaceFlags
{
public:
  bool has_pos{false};
  bool has_vel{false};
  bool has_acc{false};
  bool has_eff{false}; 
};

struct WheelState
{
public:
  // state of steering wheel module
  Double2 direction{1, 0};
  double  velocity{0};
  // state of steering and driving joint
  double  st_pos{0};
  double  st_vel{0};
  double  st_acc{0};
  double  dr_vel{0};
  double  dr_acc{0};

  WheelState() = default;
  WheelState(WheelState const&) = default;
  
  WheelState& operator= (const WheelState& other){
    if(this != &other)
    {
      direction[0] = other.direction[0];
      direction[1] = other.direction[1];
      velocity = other.velocity;
      st_pos = other.st_pos;
      st_vel = other.st_vel;
      st_acc = other.st_acc;
      dr_vel = other.dr_vel;
      dr_acc = other.dr_acc;
    }
    return *this;
  }
};

struct BaseState
{
public:
  // pose of base on world frame
  Double2 position{0, 0};
  double  rotation{0};
  // motion state of base
  Double2 direction{1, 0};
  Double2 acc_direction{1, 0};
  double  dir_diff{0};
  double  lin_vel{0};
  double  lin_acc{0};
  double  ang_vel{0};
  double  ang_acc{0};

  Double2 icr_pos{0, 0};
  double  icr_rot{0};
  Double2 g_icr_pos{0, 0};
  double  g_icr_rot{0};

  BaseState() = default;
  BaseState(BaseState const&) = default;

  BaseState& operator=(const BaseState& other){
    if(this != &other)
    {
      icr_pos[0] = other.icr_pos[0];
      icr_pos[1] = other.icr_pos[1];
      position[0] = other.position[0];
      position[1] = other.position[1];
      g_icr_pos[0] = other.g_icr_pos[0];
      g_icr_pos[1] = other.g_icr_pos[1];
      direction[0] = other.direction[0];
      direction[1] = other.direction[1];
      acc_direction[0] = other.acc_direction[0];
      acc_direction[1] = other.acc_direction[1];
      rotation = other.rotation;
      dir_diff = other.dir_diff;
      lin_vel = other.lin_vel;
      lin_acc = other.lin_acc;
      ang_vel = other.ang_vel;
      ang_acc = other.ang_acc;
      icr_rot = other.icr_rot;
      g_icr_rot = other.g_icr_rot;
    }
    return *this;
  }
};

struct ArmState
{
public:
  std::vector<double> pose;
  std::vector<double> jnt_pos;
  std::vector<double> jnt_vel;
  std::vector<double> jnt_acc;

  void clear(){
    pose.clear();
    jnt_pos.clear();
    jnt_vel.clear();
    jnt_acc.clear();
  }
};
struct WheelData
{
public:
  std::vector<std::string> joints_name;
  std::string wheel_name;
  WheelState  last_state;
  WheelState  curr_state;
  WheelState  next_state;
  WheelState  goal_state;
  Double2     pos_on_vehicle{0, 0};
  double      radius{0};
  double      halted_threshold{0};
  double      forward_time{0};
  double      reverse_time{0};
  bool        have_to_stop{false};
  bool        want_to_stop{false};
  bool        has_slippage{false};
  bool        is_halted{false};
  JointLimits swerve_joint_limits;
  JointLimits wheel_joint_limits;
};

struct BaseData
{
public:
  std::string prefix;
  BaseState last_state;
  BaseState curr_state;
  BaseState next_state;
  BaseState goal_state;
  double    dir_vel_max{0};
  double    ang_vel_max{0};
  double    dir_acc_max{0};
  double    ang_acc_max{0};
  double    dir_jerk{0};
  double    ang_jerk{0};
  bool      is_halted{false};
  JointLimits linear_limits;
  JointLimits angular_limits;
  JointLimits divergence_limits;
  std::vector<std::string> swerve_command_interfaces;
  std::vector<std::string> swerve_state_interfaces;
  std::vector<std::string> wheel_command_interfaces;
  std::vector<std::string> wheel_state_interfaces;
  std::vector<WheelData*> wheel_data_vector;
  std::map<std::string, WheelData*> wheel_data;
  InterfaceFlags swerve_interface_flags;
  InterfaceFlags wheel_interface_flags;
};

struct ArmData
{
  std::string arm_name;
  ArmState last_state;
  ArmState curr_state;
  ArmState next_state;
  ArmState goal_state;
  std::vector<std::string> joint_names;
  std::vector<std::string> state_interface_types;
  std::vector<std::string> command_interface_types;
  std::map<std::string, JointLimits*> joint_limits;
  bool is_halted{false};
  InterfaceFlags interface_flags;
};

struct HWInterfaces
{
std::vector<std::vector<std::reference_wrapper<
  hardware_interface::LoanedCommandInterface>>> command;
std::vector<std::vector<std::reference_wrapper<
  hardware_interface::LoanedStateInterface>>> state;
};
} // end of namespace mm_data_define
} // end of namespace mm_controllers
#endif  // MM_CONTROLLERS__MM_DATA_DEFINE_HPP_