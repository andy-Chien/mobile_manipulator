#include <string>
#include <Eigen/Dense>
#include <rclcpp/time.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include "mm_controllers/trajectory.hpp"
#include "mm_controllers/mm_data_define.hpp"
#include "mm_controller_parameters.hpp"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
namespace mm_controllers
{
namespace utils
{
namespace mcdd = mm_controllers::data_define;

double metersToRads(const double& meters, const double& radius);
double radsTometers(const double& rads, const double& radius);
Eigen::Vector3d calcMinimumJerkTra(
	double pos_start, double vel_start, double accel_start,
	double pos_end,   double vel_end,   double accel_end,
	double smp_time,  double mov_time);
bool updateOdometer(const rclcpp::Time& time, const rclcpp::Duration& period, 
	const std::string prefix, const bool is_halted, mcdd::BaseState& bs, 
	geometry_msgs::msg::TransformStamped& trans_msg,
	nav_msgs::msg::Odometry& odom_msg);
void recordBaseMoving(
	const rclcpp::Duration & period, const mcdd::BaseState& bs, tf2::Transform& trans);
bool checkTolerance(
  const std::vector<double>& curr_pose, const std::vector<double>& cmd_pose,
  const std::vector<double>& curr_joints, const std::vector<double>& cmd_joints);
bool checkTolerance(
  const std::vector<double>& curr_joints, const std::vector<double>& cmd_joints);
bool checkTolerance(
  const mcdd::BaseState& curr_state, const mcdd::BaseState& cmd_state);
bool checkTolerance(
  const mcdd::WheelState& curr_state, const mcdd::WheelState& cmd_state);
bool getFKResult(const std::vector<std::string>& jnt_name, const std::vector<double>& jnt_pos,
  const std::string& from, const std::string& to, moveit::core::RobotStatePtr robot_state, 
  std::vector<double>& pose);
bool getFKResult(
	const rclcpp::Duration& period, const sensor_msgs::msg::JointState& joint_state,
	rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr client,
	std::vector<double>& pose);
bool getIKResult(
  const rclcpp::Duration & time_out, const std::string& group_name, 
  const std::string& from, const std::vector<double>& pose,
  moveit::core::RobotStatePtr robot_state, std::vector<double>& jnt_pos);
bool getIKResult(
	const rclcpp::Duration & period, const std::vector<double>& pose,
	const std::string& group_name,
	rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr client, 
	sensor_msgs::msg::JointState& joint_state);
/**
 * @brief Calculate moving time using trapezoidl profile
 * @param a area
 * @param v_max max value
 * @param s slope
 * @param v0 start value
 * @param v_end end value
 * @return time in seconds
 */
double calcMovingTime(
	double a, double v_max, double s, double v_0, double v_end);
bool contains_interface_type(
	const std::vector<std::string> & interface_type_list, 
	const std::string & interface_type);
void initArmData(
	const mm_controllers::Params& params, mcdd::ArmData& arm_data);
void initBaseData(
	const mm_controllers::Params& params, mcdd::BaseData& base_data);
void initRobotState(
  const std::shared_ptr<mm_controllers::ParamListener> param_listener,
  moveit::core::RobotStatePtr& robot_state);
void ensureArmCmdLimit(
	const rclcpp::Duration period, mcdd::ArmData& arm_data);
void ensureBaseCmdLimit(
	const rclcpp::Duration& period, const mcdd::BaseData& base_data, double& dir_acc,
	double& ang_acc, mcdd::Double2& dir_cmd, double& vel_cmd, double& ang_cmd);
double ensureWheelCmdLimit(
	const rclcpp::Duration& period, mcdd::BaseData& base_data);
bool checkWheelCmdLimit(
	const rclcpp::Duration& period, mcdd::BaseData& base_data);
void setArmCmd(
  const mcdd::HWInterfaces& arm_interfaces, mcdd::ArmData& arm_data,
  const bool pos=true, const bool vel=false, const bool acc=false, const bool eff=false);
void setBaseCmd(
	const std::map<std::string, mcdd::HWInterfaces>& wheel_interfaces, mcdd::BaseData& base_data);
void armHalt(
  const rclcpp::Duration& period, const mcdd::HWInterfaces& arm_interfaces, mcdd::ArmData& arm_data,
  const bool pos=true, const bool vel=false, const bool acc=false, const bool eff=false);
void baseHalt(
	const rclcpp::Duration & period, const std::map<
	std::string, mcdd::HWInterfaces>& wheel_interfaces, 
	mcdd::BaseData& base_data);
void baseStateToJointState(
	const mcdd::BaseState& base_state, const tf2::Transform& map_t_base,
  	trajectory_msgs::msg::JointTrajectoryPoint& joint_state);
void jointStateToBaseState(
	const trajectory_msgs::msg::JointTrajectoryPoint& joint_state,
  	const tf2::Transform& map_t_odom, const tf2::Transform& map_t_base, mcdd::BaseState& base_state);
double quatToAngle(
	const geometry_msgs::msg::Quaternion& q, char axis='z');
double quatToAngle(
	const tf2::Quaternion& q, char axis='z');
void angleToQuat(
	const double angle, tf2::Quaternion& q, char axis='z');
void angleToQuat(
	const double angle, geometry_msgs::msg::Quaternion& q, char axis='z');
std::shared_ptr<std::list<Eigen::VectorXd>> convertPathToPoints(
	const nav_msgs::msg::Path & path_msg);
std::shared_ptr<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>> parameterizedToMsg(
	TrajectoryParameterization& parameterized, const double sample_time);
bool getTf(
	const std::unique_ptr<tf2_ros::Buffer>& tf_buffer,
	const std::string& from, const std::string& to, geometry_msgs::msg::TransformStamped& t,
	const rclcpp::Duration& time_out=rclcpp::Duration(0, 1e8));
void msgToTf(
	const geometry_msgs::msg::TransformStamped& t1, tf2::Transform& t2);
void msgToTf(
	const geometry_msgs::msg::Transform& t1, tf2::Transform& t2);
void msgToTf(
	const geometry_msgs::msg::PoseStamped& t1, tf2::Transform& t2);
void msgToTf(
	const geometry_msgs::msg::Pose& t1, tf2::Transform& t2);
void msgToTf(
	const geometry_msgs::msg::Quaternion& t1, tf2::Quaternion& t2);
void msgToTf(
	const geometry_msgs::msg::Vector3& t1, tf2::Vector3& t2);
void tfToMsg(
	const tf2::Transform& t1, geometry_msgs::msg::TransformStamped& t2);
void tfToMsg(
	const tf2::Transform& t1, geometry_msgs::msg::Transform& t2);
void tfToMsg(
	const tf2::Transform& t1, geometry_msgs::msg::PoseStamped& t2);
void tfToMsg(
	const tf2::Transform& t1, geometry_msgs::msg::Pose& t2);
void tfToMsg(
	const tf2::Quaternion& t1, geometry_msgs::msg::Quaternion& t2);
void tfToMsg(
	const tf2::Vector3& t1, geometry_msgs::msg::Vector3& t2);
namespace speed_limiter
{
double limit(
	double& v, double& a, const double& v_in, const double& a_in, const double& v0, 
	const double& a0, const double& dt, const mcdd::JointLimits& jl);
double limit(
	double& v, double& a, const double& v0, const double& a0, 
	const double& dt, const mcdd::JointLimits& jl);
double limit_velocity(
	double& v, const double& v0, const mcdd::JointLimits& jl);
double limit_acceleration(
	double& v, double& a, const double& v0, 
	const double& dt, const mcdd::JointLimits& jl);
double limit_jerk(
	double& v, double& a, const double& v0, const double& a0, 
	const double& dt, const mcdd::JointLimits& jl);
} // end of namespace speed_limiter
} // end of namespace mm_controllers
} // end of namespace utils