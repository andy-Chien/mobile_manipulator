#include <thread>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "mm_controllers/mm_controller_utils.hpp"

namespace mm_controllers
{
namespace utils
{
rclcpp::Logger utils_logger(rclcpp::get_logger("mm_utils"));

double metersToRads(const double& meters, const double& radius)
{
    return meters / radius;
}

double radsTometers(const double& rads, const double& radius)
{
    return rads * radius;
}

Eigen::Vector3d calcMinimumJerkTra(
    double pos_start, double vel_start, double accel_start,
    double pos_end,   double vel_end,   double accel_end,
    double smp_time,  double mov_time
    )
/*
   simple minimum jerk trajectory
   pos_start : position at initial state
   vel_start : velocity at initial state
   accel_start : acceleration at initial state
   pos_end : position at final state
   vel_end : velocity at final state
   accel_end : acceleration at final state
   smp_time : sampling time
   mov_time : movement time
 */

{
  Eigen::MatrixXd poly_matrix(3,3);
  Eigen::MatrixXd poly_vector(3,1);

  poly_matrix <<
      pow(mov_time,3), pow(mov_time,4), pow(mov_time,5),
      3*pow(mov_time,2), 4*pow(mov_time,3), 5*pow(mov_time,4),
      6*mov_time, 12*pow(mov_time,2), 20*pow(mov_time,3);

  poly_vector <<
      pos_end-pos_start-vel_start*mov_time-accel_start*pow(mov_time,2)/2,
      vel_end-vel_start-accel_start*mov_time,
      accel_end-accel_start ;

  Eigen::Matrix<double,3,1> poly_coeff = poly_matrix.inverse() * poly_vector;

  double time_steps = mov_time/smp_time;
  int all_time_steps = round(time_steps+1);

  Eigen::MatrixXd minimum_jerk_tra = Eigen::MatrixXd::Zero(all_time_steps,3);
  Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps,1);
  Eigen::Vector3d result = Eigen::Vector3d::Zero();

  result.coeffRef(0) = pos_start +
          vel_start*smp_time +
          0.5*accel_start*pow(smp_time,2) +
          poly_coeff.coeff(0,0)*pow(smp_time,3) +
          poly_coeff.coeff(1,0)*pow(smp_time,4) +
          poly_coeff.coeff(2,0)*pow(smp_time,5);
  result.coeffRef(1) = vel_start +
        accel_start*smp_time +
        3*poly_coeff.coeff(0,0)*pow(smp_time,2) +
        4*poly_coeff.coeff(1,0)*pow(smp_time,3) +
        5*poly_coeff.coeff(2,0)*pow(smp_time,4);
  result.coeffRef(2) = accel_start +
        6*poly_coeff.coeff(0,0)*smp_time +
        12*poly_coeff.coeff(1,0)*pow(smp_time,2) +
        20*poly_coeff.coeff(2,0)*pow(smp_time,3);

  return result;
}

bool updateOdometer(
  const rclcpp::Time& time, const rclcpp::Duration& period, const std::string prefix,
  const bool is_halted, mcdd::BaseState& bs, geometry_msgs::msg::TransformStamped& trans_msg,
  nav_msgs::msg::Odometry& odom_msg)
{
  /*
    do FK for base before this functon
    set and pub odometer after this function
    bs should be current base state
  */
  if(!is_halted)
  {  
    const double period_seconds = period.seconds();
    bs.rotation += bs.ang_vel * period_seconds;
    bs.rotation += 
      (bs.rotation > M_PI) ? -2 * M_PI : (bs.rotation < -1 * M_PI) ? 2 * M_PI : 0;
    bs.position[0] += (cos(bs.rotation) * bs.direction[0] * bs.lin_vel * period_seconds
                    - sin(bs.rotation) * bs.direction[1] * bs.lin_vel * period_seconds);
    bs.position[1] += (cos(bs.rotation) * bs.direction[1] * bs.lin_vel * period_seconds
                    + sin(bs.rotation) * bs.direction[0] * bs.lin_vel * period_seconds);
  }

  tf2::Quaternion q;
  q.setRPY(0, 0, bs.rotation);
  trans_msg.header.stamp = time;
  trans_msg.header.frame_id = "odom";
  trans_msg.child_frame_id = prefix + "base_footprint";

  trans_msg.transform.translation.x = bs.position[0];
  trans_msg.transform.translation.y = bs.position[1];
  trans_msg.transform.translation.z = 0.0;
  trans_msg.transform.rotation.x = q.x();
  trans_msg.transform.rotation.y = q.y();
  trans_msg.transform.rotation.z = q.z();
  trans_msg.transform.rotation.w = q.w();

  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = bs.position[0];
  odom_msg.pose.pose.position.y = bs.position[1];
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = trans_msg.transform.rotation;
  odom_msg.child_frame_id = prefix + "base_footprint";
  odom_msg.twist.twist.linear.x = bs.direction[0] * bs.lin_vel;
  odom_msg.twist.twist.linear.y = bs.direction[1] * bs.lin_vel;
  odom_msg.twist.twist.angular.z = bs.ang_vel;

  return true;
}

void recordBaseMoving(
  const rclcpp::Duration & period, const mcdd::BaseState& bs, tf2::Transform& trans)
{
  /*
    check base halted or not before this function, 
    if halted, set trans identity and no need to use this function.
    bs should be the current base state
  */
  tf2::Vector3 base_moving_pos;
  tf2::Quaternion base_moving_quat;
  tf2::Transform base_moving_trans;
  double period_seconds = period.seconds();
  base_moving_pos = tf2::Vector3(bs.direction[0] * bs.lin_vel * period_seconds, 
                                 bs.direction[1] * bs.lin_vel * period_seconds, 0);
  base_moving_quat.setRPY(0, 0, bs.ang_vel * period_seconds);
  base_moving_trans.setOrigin(base_moving_pos);
  base_moving_trans.setRotation(base_moving_quat);
  trans *= base_moving_trans;
}

bool checkTolerance(
  const std::vector<double>& curr_pose, const std::vector<double>& cmd_pose,
  const std::vector<double>& curr_joints, const std::vector<double>& cmd_joints)
{
  // make sure the last cmd has been assigned
  if(cmd_pose[0] == std::numeric_limits<double>::infinity()){
    return false;
  }
  double diff_1 = sqrt(pow((cmd_pose[0] - curr_pose[0]), 2) + 
                       pow((cmd_pose[1] - curr_pose[1]), 2) + 
                       pow((cmd_pose[2] - curr_pose[2]), 2));
  double diff_2 = sqrt(pow((cmd_pose[3] - curr_pose[3]), 2) + 
                       pow((cmd_pose[4] - curr_pose[4]), 2) + 
                       pow((cmd_pose[5] - curr_pose[5]), 2) + 
                       pow((cmd_pose[6] - curr_pose[6]), 2));
  if(diff_1 > mcdd::DEFAULT_TOLERANCE || diff_2 > mcdd::DEFAULT_TOLERANCE){
    return false;
  }
  return checkTolerance(curr_joints, cmd_joints);
}

bool checkTolerance(
  const std::vector<double>& curr_joints, const std::vector<double>& cmd_joints)
{
  for(unsigned i = 0; i < cmd_joints.size(); i++){
    if(fabs(cmd_joints[i] - curr_joints[i]) > mcdd::DEFAULT_TOLERANCE){
      return false;
    }
  }
  return true;
}

bool checkTolerance(
  const mcdd::BaseState& curr_state, const mcdd::BaseState& cmd_state)
{
  double error = curr_state.direction[0] * cmd_state.direction[0] + 
                     curr_state.direction[1] * cmd_state.direction[1]; 
  if(1.0 - error > mcdd::DEFAULT_TOLERANCE){
    RCLCPP_WARN(utils_logger,
      "base direction error out of tolerance, theta = %f", acos(error));
    return false;
  }
  error = fabs(cmd_state.lin_vel - curr_state.lin_vel);
  if(error > mcdd::DEFAULT_TOLERANCE){
    RCLCPP_WARN(utils_logger,
      "base linear velocity error out of tolerance, error = %f", error);
    return false;
  }
  error = fabs(cmd_state.ang_vel - curr_state.ang_vel);
  if(error > mcdd::DEFAULT_TOLERANCE){
    RCLCPP_WARN(utils_logger,
      "base angular velocity error out of tolerance, error = %f", error);
    return false;
  }
  return true;
}

bool checkTolerance(
  const mcdd::WheelState& curr_state, const mcdd::WheelState& cmd_state)
{
  double error = fabs(cmd_state.st_pos - curr_state.st_pos);
  if(error > mcdd::DEFAULT_TOLERANCE){
    RCLCPP_WARN(utils_logger,
      "wheel steering pos error out of tolerance, error = %f", error);
    return false;
  }
  error = fabs(cmd_state.dr_vel - curr_state.dr_vel);
  if(error > mcdd::DEFAULT_TOLERANCE){
    RCLCPP_WARN(utils_logger,
      "wheel driving velocity error out of tolerance, error = %f", error);
    return false;
  }
  return true;
}

bool getFKResult(const std::vector<std::string>& jnt_name, const std::vector<double>& jnt_pos,
  const std::string& from, const std::string& to, moveit::core::RobotStatePtr robot_state, 
  std::vector<double>& pose)
{
  try
  {
    for(size_t i=0; i<jnt_name.size(); i++){
      robot_state->setJointPositions(jnt_name[i], &(jnt_pos[i]));
    }
    robot_state->update();
    Eigen::Isometry3d to_pose = robot_state->getGlobalLinkTransform(to);

    const std::string& default_frame = robot_state->getRobotModel()->getModelFrame();
    if(default_frame.compare(from) != 0)
    {
      const Eigen::Isometry3d from_pose = robot_state->getGlobalLinkTransform(from);
      to_pose = from_pose.inverse() * to_pose;
    }
    const Eigen::Quaterniond q(to_pose.rotation());
    const Eigen::Vector3d v(to_pose.translation());
    if(pose.size() != 7){
      pose.resize(7);
    }
    pose[0] = v(0);
    pose[1] = v(1);
    pose[2] = v(2);
    pose[3] = q.w();
    pose[4] = q.x();
    pose[5] = q.y();
    pose[6] = q.z();
    return true;
  }
  catch(const std::exception& e){
    RCLCPP_WARN(utils_logger, "Failed to compute FK service.");
    return false;
  }
}

bool getFKResult(
  const rclcpp::Duration& period, const sensor_msgs::msg::JointState& joint_state,
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::
  SharedPtr client, std::vector<double>& pose)
{
  if(!client->service_is_ready())
  {
    RCLCPP_WARN(utils_logger, "FK service not started");
    return false;
  }

  auto fk_req = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  fk_req->header = joint_state.header;
  fk_req->fk_link_names.push_back("tool_tip");
  fk_req->robot_state.joint_state = joint_state;
  try
  {
    std::shared_future<std::shared_ptr<
      moveit_msgs::srv::GetPositionFK_Response>> response_future;
    response_future = client->async_send_request(fk_req).future.share();

    // wait for the service to respond
    std::chrono::nanoseconds wait_time((period * 0.2).nanoseconds());
    std::future_status fs = response_future.wait_for(wait_time);
    if(fs == std::future_status::timeout)
    {
      RCLCPP_WARN(utils_logger, "FK service timed out.");
      return false;
    }
    else
    {
      std::shared_ptr<moveit_msgs::srv::GetPositionFK_Response> response;
      response = response_future.get();
      if(response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        const auto & res_pose = response->pose_stamped[0].pose;
        pose[0] = res_pose.position.x;
        pose[1] = res_pose.position.y;
        pose[2] = res_pose.position.z;
        pose[3] = res_pose.orientation.w;
        pose[4] = res_pose.orientation.x;
        pose[5] = res_pose.orientation.y;
        pose[6] = res_pose.orientation.z;
      }
      else{
        RCLCPP_WARN(utils_logger, "FK failed.");
        return false;
      }
    }
  }
  catch(const std::exception& e){
    RCLCPP_WARN(utils_logger, "Failed to call FK service.");
    return false;
  }
  return true;
}

bool getIKResult(
  const rclcpp::Duration & time_out, const std::string& group_name, 
  const std::string& from, const std::vector<double>& pose,
  moveit::core::RobotStatePtr robot_state, std::vector<double>& jnt_pos)
{
  tf2::Transform req_pose(
    tf2::Quaternion(pose[4], pose[5], pose[6], pose[3]),
    tf2::Vector3(pose[0], pose[1], pose[2])
  );

  const moveit::core::JointModelGroup* jmg = robot_state->getJointModelGroup(group_name);
  if (!jmg){
    return false;
  }

  const std::string& default_frame = robot_state->getRobotModel()->getModelFrame();

  if(default_frame.compare(from) != 0)
  {
    const Eigen::Isometry3d from_pose = robot_state->getGlobalLinkTransform(from);
    const Eigen::Quaterniond q(from_pose.rotation());
    const Eigen::Vector3d v(from_pose.translation());
    const tf2::Transform from_pose_tf2(
      tf2::Quaternion(q.x(), q.y(), q.z(), q.w()),
      tf2::Vector3(v(0), v(1), v(2))
    );
    req_pose = from_pose_tf2 * req_pose;
  }
  geometry_msgs::msg::Pose req_pose_msg;
  tfToMsg(req_pose, req_pose_msg);
  if(robot_state->setFromIK(
    jmg, req_pose_msg, time_out.seconds(), moveit::core::GroupStateValidityCallbackFn()))
  {
    sensor_msgs::msg::JointState joint_state;
    moveit::core::robotStateToJointStateMsg(*robot_state, joint_state);
    jnt_pos = joint_state.position;
    return true;
  }
  else{
    return false;
  }
}

bool getIKResult(
  const rclcpp::Duration& period, const std::vector<double>& pose, const std::string& group_name,
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::
  SharedPtr client, sensor_msgs::msg::JointState& joint_state)
{
  if(!client->service_is_ready())
  {
    RCLCPP_WARN(utils_logger, "IK service not started");
    return false;
  }

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header = joint_state.header;
  target_pose.pose.position.x = pose[0];
  target_pose.pose.position.y = pose[1];
  target_pose.pose.position.z = pose[2];
  target_pose.pose.orientation.w = pose[3];
  target_pose.pose.orientation.x = pose[4];
  target_pose.pose.orientation.y = pose[5];
  target_pose.pose.orientation.z = pose[6];

  auto ik_req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  ik_req->ik_request.group_name = group_name;
  ik_req->ik_request.robot_state.joint_state = joint_state;
  ik_req->ik_request.pose_stamped = target_pose;
  ik_req->ik_request.avoid_collisions = true;
  ik_req->ik_request.timeout = period * 0.1;
  for(unsigned i = 0; i < joint_state.name.size(); i++)
  {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = joint_state.name[i];
    constraint.position = joint_state.position[i];
    constraint.tolerance_above = 0.1;
    constraint.tolerance_below = 0.1;
    constraint.weight = 0.1;
    ik_req->ik_request.constraints.joint_constraints.push_back(constraint);
  }
  try
  {
    std::shared_future<std::shared_ptr<
      moveit_msgs::srv::GetPositionIK_Response>> response_future;
    response_future = client->async_send_request(ik_req).future.share();

    // wait for the service to respond
    std::chrono::nanoseconds wait_time((period * 0.2).nanoseconds());
    std::future_status fs = response_future.wait_for(wait_time);
    if(fs == std::future_status::timeout)
    {
      RCLCPP_WARN(utils_logger, "IK Service timed out.");
      return false;
    }
    else
    {
      std::shared_ptr<moveit_msgs::srv::GetPositionIK_Response> response;
      response = response_future.get();
      if(response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        const auto & res_js = response->solution.joint_state;
        for(unsigned i=0; i < joint_state.name.size(); i++)
        {
          const auto itr = std::find(
            res_js.name.begin(), res_js.name.end(), joint_state.name[i]);
          if(itr != res_js.name.cend())
          {
            joint_state.position[i] = 
              res_js.position[std::distance(res_js.name.begin(), itr)];
          }
        }
      }
      else{
        RCLCPP_WARN(utils_logger, "IK Service failed.");
        return false;
      }
    }
  }
  catch(const std::exception& e){
    RCLCPP_WARN(utils_logger, "Failed to call IK service");
    return false;
  }
  return true;
}

double calcMovingTime(
  double a, double v_max, double s, double v_0, double v_end)
{
  auto [a_, s_, v_0_, v_end_, v_max_] = 
    std::tuple<double, double, double, double, double>(
      {fabs(a), fabs(s), fabs(v_0), fabs(v_end), fabs(v_max)});
  if(v_max_ < v_0_){
    v_max_ = v_0_;
  }
  if(v_max_ < v_end_){
    v_max_ = v_end_;
  }
  const double t_before = (v_0 * a < 0) ? 2 * v_0_ / s_ : 0;
  const double t_after = (v_end * a < 0) ? 2 * v_end_ / s_ : 0;
  if(a_ > (2 * v_max*v_max - v_0*v_0 - v_end*v_end) / (2 * s_))
  {
    return t_before + ((pow(v_max_ - v_0_, 2) + pow(v_max_ - v_end_, 2)) / 
      (2 * s_ * v_max_)) + (a_ / v_max_) + t_after;
  }
  else if(a_ > fabs(v_0*v_0 - v_end*v_end) / (2 * s_))
  {
    return t_before + (sqrt(4*s_*a_ + 2*v_0*v_0 + 2*v_end*v_end) - v_0_ - v_end_) / s_ + t_after;
  }
  else
  {
    return (sqrt(2*v_0*v_0 + 2*v_end*v_end - 4*s_*a_) + 
      v_0_ + std::copysign(v_end_, v_end_ * a)) / s_;
  }
}

bool contains_interface_type(
  const std::vector<std::string>& interface_type_list, const std::string& interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), 
                   interface_type) != interface_type_list.end();
}

void initArmData(const mm_controllers::Params& params, mcdd::ArmData& arm_data)
{
  arm_data.joint_names = params.arm_joints;
  arm_data.state_interface_types = params.arm_params.state_interfaces;
  arm_data.command_interface_types = params.arm_params.command_interfaces;
  for(const auto& [joint_name, limit_param] : 
    params.arm_params.joint_limits.arm_joints_map)
  {
    mcdd::JointLimits* joint_limits = new mcdd::JointLimits;
    joint_limits->pos_max = limit_param.pos_max * M_PI / 180;
    joint_limits->pos_min = limit_param.pos_min * M_PI / 180;;
    joint_limits->vel = limit_param.velocity * M_PI / 180;
    joint_limits->acc = limit_param.acceleration * M_PI / 180;
    joint_limits->vel_max = fabs(joint_limits->vel);
    joint_limits->vel_min = -1 * fabs(joint_limits->vel);
    joint_limits->acc_max = fabs(joint_limits->acc);
    joint_limits->acc_min = -1 * fabs(joint_limits->acc);
    arm_data.joint_limits.insert(
      std::pair<std::string, mcdd::JointLimits*>(joint_name, joint_limits));
  }
}

void initBaseData(const mm_controllers::Params& params, mcdd::BaseData& base_data)
{
  base_data.is_halted = false;
  base_data.prefix = params.base_params.prefix;
  base_data.dir_vel_max = params.base_params.linear_vel;
  base_data.ang_vel_max = params.base_params.angular_vel;
  base_data.dir_acc_max = params.base_params.linear_acc;
  base_data.ang_acc_max = params.base_params.angular_acc;
  base_data.dir_jerk = params.base_params.linear_jerk;
  base_data.ang_jerk = params.base_params.angular_jerk;
  base_data.swerve_command_interfaces = params.base_params.swerve_command_interfaces;
  base_data.swerve_state_interfaces = params.base_params.swerve_state_interfaces;
  base_data.wheel_command_interfaces = params.base_params.wheel_command_interfaces;
  base_data.wheel_state_interfaces = params.base_params.wheel_state_interfaces;

  base_data.linear_limits.has_vel_limit = true;
  base_data.linear_limits.has_acc_limit = true;
  base_data.linear_limits.has_jerk_limit = true;
  base_data.linear_limits.vel_max = params.base_params.linear_vel;
  base_data.linear_limits.vel_min = -1 * params.base_params.linear_vel;
  base_data.linear_limits.acc_max = params.base_params.linear_acc;
  base_data.linear_limits.acc_min = -1 * params.base_params.linear_acc;
  base_data.linear_limits.jerk_max = params.base_params.linear_jerk;
  base_data.linear_limits.jerk_min = -1 * params.base_params.linear_jerk;

  base_data.angular_limits.has_vel_limit = true;
  base_data.angular_limits.has_acc_limit = true;
  base_data.angular_limits.has_jerk_limit = true;
  base_data.angular_limits.vel_max = params.base_params.angular_vel;
  base_data.angular_limits.vel_min = -1 * params.base_params.angular_vel;
  base_data.angular_limits.acc_max = params.base_params.angular_acc;
  base_data.angular_limits.acc_min = -1 * params.base_params.angular_acc;
  base_data.angular_limits.jerk_max = params.base_params.angular_jerk;
  base_data.angular_limits.jerk_min = -1 * params.base_params.angular_jerk;

  base_data.divergence_limits.has_vel_limit = true;
  base_data.divergence_limits.has_acc_limit = true;
  base_data.divergence_limits.vel_max = params.base_params.divergence_vel;
  base_data.divergence_limits.vel_min = -1* params.base_params.divergence_vel;
  base_data.divergence_limits.acc_max = params.base_params.divergence_acc;
  base_data.divergence_limits.acc_min = -1* params.base_params.divergence_acc;

  mcdd::WheelData* wheel_data;
  for(const auto& [wheel_name, wheel_params] : 
    params.base_params.wheel_params.wheel_names_map)
  {
    wheel_data = new mcdd::WheelData();
    wheel_data->wheel_name = wheel_name;
    wheel_data->pos_on_vehicle[0] = wheel_params.pos_on_vehicle[0];
    wheel_data->pos_on_vehicle[1] = wheel_params.pos_on_vehicle[1];
    wheel_data->radius = wheel_params.radius;
    auto joints_name = wheel_params.joints;
    for(auto jn=joints_name.begin(); jn!=joints_name.end(); ++jn){
        wheel_data->joints_name.push_back(base_data.prefix + *jn);
    }

    auto& wsl = wheel_data->swerve_joint_limits;
    auto& wwl = wheel_data->wheel_joint_limits;
    wsl.has_pos_limit = wheel_params.swerve_joint_has_pos_limit;
    wsl.has_vel_limit = wheel_params.swerve_joint_has_vel_limit;
    wsl.has_acc_limit = wheel_params.swerve_joint_has_acc_limit;
    wsl.pos_max = wheel_params.swerve_joint_pos_max * M_PI / 180; 
    wsl.pos_min = wheel_params.swerve_joint_pos_min * M_PI / 180;
    wsl.vel = wheel_params.swerve_joint_velocity_max * M_PI / 180;
    wsl.acc = wheel_params.swerve_joint_acceleration_max * M_PI / 180;

    wwl.has_vel_limit = wheel_params.wheel_joint_has_vel_limit;
    wwl.has_acc_limit = wheel_params.wheel_joint_has_acc_limit;
    wwl.vel = wheel_params.wheel_joint_velocity_max * M_PI / 180;
    wwl.acc = wheel_params.wheel_joint_acceleration_max * M_PI / 180;

    base_data.wheel_data_vector.push_back(wheel_data);
    base_data.wheel_data.insert(
      std::pair<std::string, mcdd::WheelData*>(wheel_name, wheel_data));
  }
}

void initRobotState(
  const std::shared_ptr<mm_controllers::ParamListener> param_listener,
  moveit::core::RobotStatePtr& robot_state)
{
  const auto tmp_node = std::make_shared<rclcpp::Node>("robot_state_init_node");
  mm_controllers::Params params = param_listener->get_params();
  while(params.robot_description.size() == 0 || 
        params.robot_description_semantic.size() == 0){
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    param_listener->refresh_dynamic_parameters();
    params = param_listener->get_params();
    std::cout<<"Waiting for rdf param."<<std::endl;
  }

  const std::string kinematics_prefix("robot_description_kinematics."); 

  tmp_node->declare_parameter<std::string>(
    "robot_description", params.robot_description);
  tmp_node->declare_parameter<std::string>(
    "robot_description_semantic", params.robot_description_semantic);

  tmp_node->declare_parameter<std::string>(
    kinematics_prefix + params.kinematics.arm_ik_group + ".kinematics_solver", 
    params.kinematics.kinematics_solver
  );
  tmp_node->declare_parameter<double>(
    kinematics_prefix + params.kinematics.arm_ik_group + ".kinematics_solver_search_resolution",
    params.kinematics.kinematics_solver_search_resolution
  );
  tmp_node->declare_parameter<double>(
    kinematics_prefix + params.kinematics.arm_ik_group + ".kinematics_solver_timeout", 
    params.kinematics.kinematics_solver_timeout
  );

  robot_model_loader::RobotModelLoader rml(tmp_node, "robot_description");
  robot_state = std::make_shared<moveit::core::RobotState>(rml.getModel());
  robot_state->setToDefaultValues();
}

void ensureArmCmdLimit(const rclcpp::Duration period, mcdd::ArmData& arm_data)
{
  double factor, min_factor = 1;
  const double period_seconds = period.seconds();
  for(size_t i = 0; i < arm_data.joint_names.size(); i++)
  {
    auto joint_limit = arm_data.joint_limits.at(arm_data.joint_names[i]);
    if(joint_limit->has_pos_limit && joint_limit->has_vel_limit)
    {
      const double pos_max_diff = joint_limit->pos_max - arm_data.curr_state.jnt_pos[i];
      const double v_max = sqrt(fabs(
        2 * pos_max_diff * joint_limit->acc_min)) * std::copysign(1.0, pos_max_diff);
      const double pos_min_diff = joint_limit->pos_min - arm_data.curr_state.jnt_pos[i];
      const double v_min = sqrt(fabs(
        2 * pos_min_diff * joint_limit->acc_min)) * std::copysign(1.0, pos_min_diff);

      if(v_max < joint_limit->vel_max){
        joint_limit->vel_max = v_max;
      }
      if(v_min < joint_limit->vel_min){
        joint_limit->vel_min = v_min;
      }
      if(arm_data.curr_state.jnt_vel[i] > v_max ||
         arm_data.curr_state.jnt_vel[i] > v_min)
      {
        RCLCPP_WARN(utils_logger, 
          "Arm command is close to limit with a unstoppable speed!");
      }
    }
    factor = speed_limiter::limit(
      arm_data.next_state.jnt_vel[i],
      arm_data.next_state.jnt_acc[i],
      arm_data.goal_state.jnt_vel[i],
      arm_data.goal_state.jnt_acc[i],
      arm_data.last_state.jnt_vel[i],
      arm_data.last_state.jnt_acc[i],
      period_seconds,
      *joint_limit
    );
    if(factor < min_factor){
      min_factor = factor;
    }
  }

  for(size_t i = 0; i < arm_data.joint_names.size(); i++)
  {
    if(min_factor < 1)
    {
      arm_data.next_state.jnt_acc[i] = 
        min_factor * arm_data.goal_state.jnt_acc[i];
      arm_data.next_state.jnt_vel[i] = 
        arm_data.last_state.jnt_vel[i] + arm_data.next_state.jnt_acc[i] * period_seconds;
      arm_data.next_state.jnt_pos[i] = 
        arm_data.last_state.jnt_pos[i] + arm_data.next_state.jnt_vel[i] * period_seconds;
    }else{
      arm_data.next_state.jnt_pos[i] = arm_data.goal_state.jnt_pos[i];
    }
    const auto joint_limit = arm_data.joint_limits.at(arm_data.joint_names[i]);
    if(joint_limit->has_pos_limit &&
      (arm_data.next_state.jnt_pos[i] > joint_limit->pos_max || 
       arm_data.next_state.jnt_pos[i] < joint_limit->pos_min))
    {
      if(arm_data.next_state.jnt_pos[i] > joint_limit->pos_max){
        arm_data.next_state.jnt_pos[i] = joint_limit->pos_max;
      }
      else if(arm_data.next_state.jnt_pos[i] < joint_limit->pos_min){
        arm_data.next_state.jnt_pos[i] = joint_limit->pos_min;
      }
      arm_data.next_state.jnt_vel[i] = 
        (arm_data.next_state.jnt_pos[i] - arm_data.last_state.jnt_pos[i]) / period_seconds;
      arm_data.next_state.jnt_acc[i] = 
        (arm_data.next_state.jnt_vel[i] - arm_data.last_state.jnt_vel[i]) / period_seconds;
      RCLCPP_ERROR(utils_logger, 
        "Arm command position out of range!");
    }
  }
}

// void ensureBaseCmdLimit(const rclcpp::Duration period, mcdd::BaseData& base_data)
// {
//   auto bll = base_data.linear_limits;
//   auto bal = base_data.angular_limits;
//   auto bdl = base_data.divergence_limits;
//   double factor, min_factor = 1;
//   const double period_seconds = period.seconds();

//   factor = speed_limiter::limit(
//     base_data.next_state.lin_vel,
//     base_data.next_state.lin_acc,
//     base_data.goal_state.lin_vel,
//     base_data.goal_state.lin_acc,
//     base_data.last_state.lin_vel,
//     base_data.last_state.lin_acc,
//     period_seconds,
//     base_data.linear_limits
//   );
//   if(factor < min_factor){
//     min_factor = factor;
//   }
//   factor = speed_limiter::limit(
//     base_data.next_state.ang_vel,
//     base_data.next_state.ang_acc,
//     base_data.goal_state.ang_vel,
//     base_data.goal_state.ang_acc,
//     base_data.last_state.ang_vel,
//     base_data.last_state.ang_acc,
//     period_seconds,
//     base_data.linear_limits
//   );
//   if(factor < min_factor){
//     min_factor = factor;
//   }
//   const mcdd::Double2 div_goal_vel = {
//     base_data.goal_state.direction[0] - base_data.last_state.direction[0],
//     base_data.goal_state.direction[1] - base_data.last_state.direction[1]
//   }
//   const mcdd::Double2 div_last = {
//     base_data.goal_state.direction[0] - base_data.last_state.direction[0],
//     base_data.goal_state.direction[1] - base_data.last_state.direction[1]
//   }
//   mcdd::Double2 div_next;

//   factor = speed_limiter::limit(
//     base_data.next_state.lin_vel,
//     base_data.next_state.lin_acc,
//     base_data.goal_state.lin_vel,
//     base_data.goal_state.lin_acc,
//     base_data.last_state.lin_vel,
//     base_data.last_state.lin_acc,
//     period_seconds,
//     base_data.linear_limits
//   );
//   if(factor < min_factor){
//     min_factor = factor;
//   }
// }

void ensureBaseCmdLimit(
  const rclcpp::Duration& period, const mcdd::BaseData& base_data, double& dir_acc,
  double& ang_acc, mcdd::Double2& dir_cmd, double& vel_cmd, double& ang_cmd)
{
  const double& d_j = base_data.dir_jerk;
  const double& a_j = base_data.ang_jerk;
  double period_seconds = period.seconds();
  double dir_acc_diff = d_j * period_seconds;
  double ang_acc_diff = a_j * period_seconds;
  double ang_diff, norm_cmd_curr;
  const mcdd::BaseState& gs = base_data.goal_state;
  const mcdd::BaseState& ls = base_data.last_state;
  mcdd::Double2 dir_diff = 
    {gs.direction[0] * gs.lin_vel - ls.direction[0] * ls.lin_vel,
     gs.direction[1] * gs.lin_vel - ls.direction[1] * ls.lin_vel};
  norm_cmd_curr = sqrt(pow(dir_diff[0], 2) + pow(dir_diff[1], 2));
  ang_diff = gs.ang_vel - ls.ang_vel;

  bool update_dir, update_ang;
  double d_ta, d_tc, d_td, a_ta, a_tc, a_td; //time acceleration, continous, deceleration.
  double d_v_diff, a_v_diff; // velocity difference for dir and angle
  double d_a0 = ls.lin_acc;
  double a_a0 = ls.ang_acc;
  d_td = std::ceil(fabs(d_a0) / dir_acc_diff);
  a_td = std::ceil(fabs(a_a0) / ang_acc_diff);
  d_v_diff = d_a0 * period_seconds * (1 + d_td / 2);
  a_v_diff = a_a0 * period_seconds * (1 + a_td / 2);
  if(fabs(d_v_diff) - norm_cmd_curr > dir_acc_diff)
  {
    update_dir = false;
    dir_acc_diff = d_a0 / d_td;
    dir_acc = d_a0 - dir_acc_diff;
    d_tc = 0;
    d_ta = 0;
  }
  else
  {
    update_dir = true;
    d_v_diff = (dir_acc * dir_acc - (d_a0 * d_a0 / 2)) / d_j;
    if(d_v_diff < norm_cmd_curr) // tc != 0
    {
      d_tc = norm_cmd_curr / dir_acc - dir_acc / d_j + (d_a0 * d_a0) / (2 * d_j * dir_acc);
      d_td = dir_acc / d_j;
      d_ta = d_td - fabs(d_a0) / d_j;
    }
    else
    {
      d_tc = 0;
      d_td = sqrt(norm_cmd_curr / d_j + (d_a0 * d_a0) / (2 * d_j * d_j));
      d_ta = d_td - fabs(d_a0) / d_j;
    }
  }

  if(fabs(a_v_diff) - fabs(ang_diff) > ang_acc_diff)
  {
    update_ang = false;
    ang_acc_diff = a_a0 / a_td;
    ang_acc = a_a0 - ang_acc_diff;
    a_tc = 0;
    a_ta = 0;
  }
  else
  {
    update_ang = true;
    a_v_diff = (ang_acc * ang_acc - (a_a0 * a_a0 / 2)) / a_j;
    if(a_v_diff < fabs(ang_diff)) // tc != 0
    {
      a_tc = fabs(ang_diff) / ang_acc - ang_acc / a_j + (a_a0 * a_a0) / (2 * a_j * ang_acc);
      a_td = ang_acc / a_j;
      a_ta = a_td - fabs(a_a0) / a_j;
    }
    else
    {
      a_tc = 0;
      a_td = sqrt(fabs(ang_diff) / a_j + (a_a0 * a_a0) / (2 * a_j * a_j));
      a_ta = a_td - fabs(a_a0) / a_j;
    }
  }
  double ta = (d_ta > a_ta) ? std::ceil(d_ta / period_seconds) : std::ceil(a_ta / period_seconds);
  double tc = (d_tc > a_tc) ? std::ceil(d_tc / period_seconds) : std::ceil(a_tc / period_seconds);
  double td = (d_td > a_td) ? std::ceil(d_td / period_seconds) : std::ceil(a_td / period_seconds);
  if(ta + tc + td > 1)
  {
    if(update_dir)
    {
      dir_acc = (2 * norm_cmd_curr / period_seconds - ta * fabs(d_a0)) / (ta + 2 * tc + td);
      dir_acc_diff = (ta > 0) ? (dir_acc - d_a0) / ta : 0;
      dir_acc = d_a0 + dir_acc_diff;
    }
    if(update_ang)
    {
      ang_acc = (2 * fabs(ang_diff) / period_seconds - ta * fabs(a_a0)) / (ta + 2 * tc + td);
      ang_acc_diff = (ta > 0) ? (ang_acc - a_a0) / ta : 0;
      ang_acc = a_a0 + ang_acc_diff;
    }
  }
  else
  {
    dir_acc = 0;
    ang_acc = 0;
  }
  double dir_diff_max = fabs(dir_acc * period_seconds);
  double ang_diff_max = ang_acc * period_seconds;

  if(dir_diff_max > 0 && norm_cmd_curr > 0)
  {
    double vel_x, vel_y;
    vel_x = ls.direction[0] * ls.lin_vel + dir_diff[0] * dir_diff_max / norm_cmd_curr;
    vel_y = ls.direction[1] * ls.lin_vel + dir_diff[1] * dir_diff_max / norm_cmd_curr;
    vel_cmd = std::hypot(vel_x, vel_y);
    if(vel_cmd > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT){
      dir_cmd[0] = vel_x / vel_cmd;
      dir_cmd[1] = vel_y / vel_cmd;
    }else{
      dir_cmd[0] = ls.direction[0];
      dir_cmd[1] = ls.direction[1];
    }
  }
  else
  {
    vel_cmd = gs.lin_vel;
    dir_cmd[0] = gs.direction[0];
    dir_cmd[1] = gs.direction[1];
  }
  if(fabs(ang_diff_max) > 0 && fabs(ang_diff) > 0)
  {
    ang_cmd = ls.ang_vel + std::copysign(ang_diff_max, ang_diff);
  }
  else
  {
    ang_cmd = gs.ang_vel;
  }
}

bool checkWheelCmdLimit(const rclcpp::Duration& period, mcdd::BaseData& base_data)
{
  const double period_second = period.seconds();
  bool success = true;
  for(auto& wheel_data : base_data.wheel_data_vector)
  {
    wheel_data->next_state.st_vel = 
      (wheel_data->next_state.st_pos - wheel_data->last_state.st_pos) / period_second;

    if(fabs(wheel_data->next_state.st_vel) > wheel_data->swerve_joint_limits.vel){
      std::cout<<"1, "<<wheel_data->next_state.st_vel<<", "<<wheel_data->next_state.st_pos<<", "<<wheel_data->last_state.st_pos<<", ";
      success = false;
    }
    wheel_data->next_state.st_acc = 
      (wheel_data->next_state.st_vel - wheel_data->last_state.st_vel) / period_second;

    if(fabs(wheel_data->next_state.st_acc) > wheel_data->swerve_joint_limits.acc){
      std::cout<<"2, "<<wheel_data->next_state.st_acc<<", "<<wheel_data->next_state.st_vel<<", "<<wheel_data->last_state.st_vel<<", ";
      success = false;
    }
    if(fabs(wheel_data->next_state.dr_vel) > wheel_data->wheel_joint_limits.vel){
      std::cout<<"3"<<wheel_data->next_state.dr_vel<<", ";
      success = false;
    }
    wheel_data->next_state.dr_acc = 
      (wheel_data->next_state.dr_vel - wheel_data->last_state.dr_vel) / period_second;

    if(fabs(wheel_data->next_state.dr_acc) > wheel_data->wheel_joint_limits.acc){
      std::cout<<"4"<<wheel_data->next_state.dr_acc<<", ";
      success = false;
    }
  }
  return success;
}

double ensureWheelCmdLimit(const rclcpp::Duration& period, mcdd::BaseData& base_data)
{
  const double period_second = period.seconds();

  double swerve_pos_diff, swerve_vel_diff, wheel_vel_diff, swerve_tar_pos_diff;
  double s_vel_scale, s_vel_scale_min = 1;
  double s_acc_time, s_acc_time_max = 1;
  double w_vel_scale, w_vel_scale_min = 1;
  double w_acc_time, w_acc_time_max = 1;
  std::vector<double> orig_s_vel;
  std::vector<double> orig_w_vel;
  // std::cout<<"==================="<<std::endl;
  for(auto& wheel_data : base_data.wheel_data_vector)
  {
    const double swerve_vel_limit = wheel_data->swerve_joint_limits.vel;
    const double swerve_vel_diff_limit = wheel_data->swerve_joint_limits.acc * period_second;
    const double wheel_vel_limit = wheel_data->wheel_joint_limits.vel;
    swerve_pos_diff = wheel_data->next_state.st_pos - wheel_data->last_state.st_pos;
    swerve_tar_pos_diff = wheel_data->goal_state.st_pos - wheel_data->last_state.st_pos;
    double stop_time = std::ceil(fabs(wheel_data->last_state.st_vel) / swerve_vel_diff_limit);
    double stop_diff = fabs(wheel_data->last_state.st_vel) * period_second * (1 + stop_time / 2);
    wheel_data->next_state.st_vel = swerve_pos_diff / period_second;
    orig_s_vel.push_back(fabs(wheel_data->next_state.st_vel));
    orig_w_vel.push_back(fabs(wheel_data->next_state.dr_vel));
    double s_vel_limit = swerve_vel_limit;
    if(stop_diff - fabs(swerve_tar_pos_diff) > swerve_vel_diff_limit)
    {
      double deacc = wheel_data->last_state.st_vel / stop_time;
      s_vel_limit = wheel_data->last_state.st_vel - deacc;
      // std::cout<<wheel_data->wheel_name<<
      // " should slow down steering! last_state.st_vel = "<<
      // wheel_data->last_state.st_vel<<", stop_diff = "<<stop_diff<<
      // ", swerve_tar_pos_diff = "<<fabs(swerve_tar_pos_diff)<<std::endl;
    }
    if(fabs(wheel_data->next_state.st_vel) > fabs(s_vel_limit)){
      s_vel_scale = fabs(s_vel_limit / wheel_data->next_state.st_vel);
      if(s_vel_scale < s_vel_scale_min){
        s_vel_scale_min = s_vel_scale;
      }
      // std::cout<<wheel_data->wheel_name<<" swerve velocity out of limited"<<std::endl;
    }
    
    if(fabs(wheel_data->next_state.dr_vel) > wheel_vel_limit)
    {
      w_vel_scale = wheel_vel_limit / fabs(wheel_data->next_state.dr_vel);
      if(w_vel_scale < w_vel_scale_min){
        w_vel_scale_min = w_vel_scale;
      }
      // std::cout<<wheel_data->wheel_name<<" wheel velocity out of limited"<<std::endl;
    }
  }
  double vel_scale = (s_vel_scale_min < w_vel_scale_min) ? s_vel_scale_min : w_vel_scale_min;
  for(auto& wheel_data : base_data.wheel_data_vector)
  {
    const double swerve_vel_diff_limit = wheel_data->swerve_joint_limits.acc * period_second;
    const double wheel_vel_diff_limit = wheel_data->wheel_joint_limits.acc * period_second;
    wheel_data->next_state.st_vel *= s_vel_scale_min;
    wheel_data->next_state.dr_vel *= w_vel_scale_min;
    swerve_vel_diff = fabs(wheel_data->next_state.st_vel - wheel_data->last_state.st_vel);
    if(swerve_vel_diff > swerve_vel_diff_limit){
      s_acc_time = swerve_vel_diff / swerve_vel_diff_limit;
      if(s_acc_time > s_acc_time_max){
        s_acc_time_max = s_acc_time;
      }
      // std::cout<<wheel_data->wheel_name<<" swerve acceleration out of limited"<<std::endl;
    }
    wheel_vel_diff = fabs(wheel_data->next_state.dr_vel - wheel_data->last_state.dr_vel);
    if(wheel_vel_diff > wheel_vel_diff_limit)
    {
      w_acc_time = wheel_vel_diff / wheel_vel_diff_limit;
      if(w_acc_time > w_acc_time_max){
        w_acc_time_max = w_acc_time;
      }
      // std::cout<<wheel_data->wheel_name<<" wheel acceleration out of limited"<<std::endl;
    }
  }
  for(auto& wheel_data : base_data.wheel_data_vector)
  {
    swerve_vel_diff = 
      wheel_data->next_state.st_vel - wheel_data->last_state.st_vel;
    wheel_vel_diff = 
      wheel_data->next_state.dr_vel - wheel_data->last_state.dr_vel;
    wheel_data->next_state.st_vel = 
      wheel_data->last_state.st_vel + swerve_vel_diff / s_acc_time_max;
    swerve_pos_diff = 
      wheel_data->next_state.st_vel * period_second;
    wheel_data->next_state.st_pos = 
      wheel_data->last_state.st_pos + swerve_pos_diff;
    wheel_data->next_state.dr_vel = 
      wheel_data->last_state.dr_vel + wheel_vel_diff / w_acc_time_max;
  }
  vel_scale = 1;
  for(size_t i=0; i<base_data.wheel_data_vector.size(); i++)
  {
    mcdd::WheelState& ws = base_data.wheel_data_vector[i]->next_state;
    double s_scale = fabs(ws.st_vel) / orig_s_vel[i];
    double w_scale = fabs(ws.dr_vel) / orig_w_vel[i];
    if(s_scale < vel_scale)
      vel_scale = s_scale;
    if(w_scale < vel_scale)
      vel_scale = w_scale;
  }
  // std::cout<<"+++++++++++++++ vel_scale = "<<vel_scale<<std::endl;
  return vel_scale;
}

void setArmCmd(
  const mcdd::HWInterfaces& arm_interfaces, mcdd::ArmData& arm_data,
  const bool pos, const bool vel, const bool acc, const bool eff)
{
  auto assign_interface_from_point =
    [&](const auto& joint_interface, const std::vector<double> & cmd){
      for(size_t i = 0; i < arm_data.joint_names.size(); i++){
        joint_interface[i].get().set_value(cmd[i]);
      }
    };

  if(pos){
    assign_interface_from_point(
      arm_interfaces.command[0], arm_data.next_state.jnt_pos);
    arm_data.is_halted = true;
    for(size_t i = 0; i < arm_data.joint_names.size(); i++){
      arm_data.is_halted = arm_data.is_halted && (fabs(
        arm_data.next_state.jnt_pos[i] - arm_data.last_state.jnt_pos[i]) <
        mcdd::DEFAULT_SMALL_ENOUGH_FLOAT);
    }
  }
  if(vel){
    assign_interface_from_point(
      arm_interfaces.command[1], arm_data.next_state.jnt_vel);
    arm_data.is_halted = true;
    for(size_t i = 0; i < arm_data.joint_names.size(); i++){
      arm_data.is_halted = arm_data.is_halted && (fabs(
        arm_data.next_state.jnt_vel[i]) < mcdd::DEFAULT_HALTED_THRESHOLD);
    }
  }
  if(acc){
    RCLCPP_ERROR(utils_logger,
      "Acceleration command is not support currently");
  }
  if(eff){
    RCLCPP_ERROR(utils_logger,
      "Acceleration command is not support currently");
  }
  arm_data.last_state = arm_data.next_state;
}

void setBaseCmd(
  const std::map<std::string, mcdd::HWInterfaces>& wheel_interfaces, mcdd::BaseData& base_data)
{
  base_data.is_halted = true;
  for(auto const& [wheel_name, wheel_handle] : wheel_interfaces)
  {
    auto wheel_data = base_data.wheel_data[wheel_name];
    wheel_handle.command[0][0].get().set_value(wheel_data->next_state.st_pos);
    wheel_handle.command[1][0].get().set_value(wheel_data->next_state.dr_vel);
    wheel_data->last_state = wheel_data->next_state;
    if(fabs(wheel_data->next_state.dr_vel) < mcdd::DEFAULT_HALTED_THRESHOLD){
      wheel_data->is_halted = true;
    }else{
      wheel_data->is_halted = false;
    }
    base_data.is_halted = base_data.is_halted && wheel_data->is_halted;
  }
  base_data.last_state = base_data.next_state;
}

void armHalt(
  const rclcpp::Duration& period, const mcdd::HWInterfaces& arm_interfaces, 
  mcdd::ArmData& arm_data, const bool pos, const bool vel, const bool acc, const bool eff)
{
  if(checkTolerance(arm_data.curr_state.jnt_pos, arm_data.last_state.jnt_pos)){
    arm_data.goal_state = arm_data.last_state;
  }else{
    arm_data.goal_state = arm_data.curr_state;
  }

  const double period_seconds = period.seconds();

  for(size_t i = 0; i < arm_data.joint_names.size(); i++)
  {
    arm_data.goal_state.jnt_vel[i] = 0;
    arm_data.goal_state.jnt_acc[i] = 
      -1 * arm_data.last_state.jnt_vel[i] / period_seconds;
  }
  ensureArmCmdLimit(period, arm_data);
  setArmCmd(arm_interfaces, arm_data, pos, vel, acc, eff);
}

void baseHalt(
  const rclcpp::Duration & period, const std::map<
  std::string, mcdd::HWInterfaces>& wheel_interfaces, mcdd::BaseData& base_data)
{
  double s_stoping_acc, w_stoping_acc;
  double s_max_stoping_time = 0;
  double w_max_stoping_time = 0;
  for(auto & [wheel_name, wheel_data] : base_data.wheel_data)
  {
    double s_stoping_time = fabs(
      wheel_data->curr_state.st_vel / wheel_data->swerve_joint_limits.acc);
    double w_stoping_time = fabs(
      wheel_data->curr_state.dr_vel / wheel_data->wheel_joint_limits.acc);
    if(s_stoping_time > s_max_stoping_time)
    {
      s_max_stoping_time = s_stoping_time;
    }
    if(w_stoping_time > w_max_stoping_time)
    {
      w_max_stoping_time = w_stoping_time;
    }
  }
  double period_seconds = period.seconds();
  bool s_last_stop_cmd = s_max_stoping_time < period_seconds;
  bool w_last_stop_cmd = w_max_stoping_time < period_seconds;
  for(auto & [wheel_name, wheel_data] : base_data.wheel_data)
  {
    if(s_last_stop_cmd){
      wheel_data->next_state.st_vel = 0;
    }else{
      s_stoping_acc = wheel_data->last_state.st_vel / s_max_stoping_time;
      wheel_data->next_state.st_vel = 
        wheel_data->last_state.st_vel - s_stoping_acc * period_seconds;
    }
    if(w_last_stop_cmd){
      wheel_data->next_state.dr_vel = 0;
    }else{
      w_stoping_acc = wheel_data->curr_state.dr_vel / w_max_stoping_time;
      wheel_data->next_state.dr_vel = 
        wheel_data->curr_state.dr_vel - w_stoping_acc * period_seconds;
    }
    wheel_data->next_state.st_pos = 
      wheel_data->last_state.st_pos + wheel_data->next_state.st_vel * period_seconds;
  }
  if(w_last_stop_cmd)
  {
    base_data.next_state.lin_vel = 0;
    base_data.next_state.lin_acc = 0;
    base_data.next_state.ang_vel = 0;
    base_data.next_state.ang_acc = 0;
  }
  else
  {
    base_data.next_state.direction[0] = base_data.curr_state.direction[0];
    base_data.next_state.direction[1] = base_data.curr_state.direction[1];
    base_data.next_state.lin_vel = std::trunc(base_data.curr_state.lin_vel * 100) / 100.0;
    base_data.next_state.ang_vel = std::trunc(base_data.curr_state.ang_vel * 100) / 100.0;
  }
  base_data.next_state.position[0] = base_data.curr_state.position[0];
  base_data.next_state.position[1] = base_data.curr_state.position[1];
  base_data.next_state.rotation = base_data.curr_state.rotation;

  setBaseCmd(wheel_interfaces, base_data);
}

void baseStateToJointState(const mcdd::BaseState& base_state, const tf2::Transform& map_t_base,
  trajectory_msgs::msg::JointTrajectoryPoint& joint_state)
{
  assert(joint_state.positions.size() >= 3 && 
         joint_state.velocities.size() >= 3 && 
         joint_state.accelerations.size() >= 3);

  const double curr_ang = quatToAngle(map_t_base.getRotation());
  const double sin_theta = sin(curr_ang);
  const double cos_theta = cos(curr_ang);
  const mcdd::Double2 v_dir = {
    cos_theta * base_state.direction[0] - sin_theta * base_state.direction[1],
    cos_theta * base_state.direction[1] + sin_theta * base_state.direction[0]
    };
  const mcdd::Double2 a_dir = {
    cos_theta * base_state.acc_direction[0] - sin_theta * base_state.acc_direction[1],
    cos_theta * base_state.acc_direction[1] + sin_theta * base_state.acc_direction[0]
    };
  joint_state.positions[0] = map_t_base.getOrigin().getX();
  joint_state.positions[1] = map_t_base.getOrigin().getY();
  joint_state.positions[2] = curr_ang;
  joint_state.velocities[0] = v_dir[0] * base_state.lin_vel;
  joint_state.velocities[1] = v_dir[1] * base_state.lin_vel;
  joint_state.velocities[2] = base_state.ang_vel;
  joint_state.accelerations[0] = a_dir[0] * base_state.lin_acc;
  joint_state.accelerations[1] = a_dir[1] * base_state.lin_acc;
  joint_state.accelerations[2] = base_state.ang_acc;
}

void jointStateToBaseState(const trajectory_msgs::msg::JointTrajectoryPoint& joint_state,
  const tf2::Transform& map_t_odom, const tf2::Transform& map_t_base, mcdd::BaseState& base_state)
{
  assert(joint_state.positions.size() >= 3 && 
         joint_state.velocities.size() >= 3 && 
         joint_state.accelerations.size() >= 3);

  const tf2::Transform odom_t_base = map_t_odom.inverse() * map_t_base;
  const double curr_ang = quatToAngle(map_t_base.getRotation());
  const double sin_theta = sin(curr_ang);
  const double cos_theta = cos(curr_ang);
  const mcdd::Double2 v_vec = {
    cos_theta * joint_state.velocities[0] + sin_theta * joint_state.velocities[1],
    cos_theta * joint_state.velocities[1] - sin_theta * joint_state.velocities[0]
    };
  const mcdd::Double2 a_vec = {
    cos_theta * joint_state.accelerations[0] + sin_theta * joint_state.accelerations[1],
    cos_theta * joint_state.accelerations[1] - sin_theta * joint_state.accelerations[0]
    };
  base_state.position[0] = odom_t_base.getOrigin().getX();
  base_state.position[1] = odom_t_base.getOrigin().getY();
  base_state.rotation = quatToAngle(odom_t_base.getRotation());
  base_state.lin_vel = std::hypot(v_vec[0], v_vec[1]);
  base_state.ang_vel = joint_state.velocities[2];
  base_state.lin_acc = std::hypot(a_vec[0], a_vec[1]);
  base_state.ang_acc = joint_state.accelerations[2];
  if(base_state.lin_vel > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
  {
    base_state.direction[0] = v_vec[0] / base_state.lin_vel;
    base_state.direction[1] = v_vec[1] / base_state.lin_vel;
  }
  if(base_state.lin_acc > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
  {
    base_state.acc_direction[0] = a_vec[0] / base_state.lin_acc;
    base_state.acc_direction[1] = a_vec[1] / base_state.lin_acc;
  }
}

double quatToAngle(const geometry_msgs::msg::Quaternion& q, char axis)
{
  return quatToAngle(tf2::Quaternion(q.x, q.y, q.z, q.w), axis);
}

double quatToAngle(const tf2::Quaternion& q, char axis)
{
  double angle = NAN;
  tf2::Vector3 tar_axis;
  tf2::Matrix3x3 mat(q);

  if(axis == 'x' || axis == 'X'){
    tar_axis = mat.getColumn(1);
    if(fabs(tar_axis.getX()) < 0.9){
      angle = atan2(tar_axis.getZ(), tar_axis.getY());
    }else{
      tar_axis = mat.getColumn(2);
      angle = atan2(-1*tar_axis.getY(), tar_axis.getZ());
    }
  }else if(axis == 'y' || axis == 'Y'){
    tar_axis = mat.getColumn(2);
    if(fabs(tar_axis.getY()) < 0.9){
      angle = atan2(tar_axis.getX(), tar_axis.getZ());
    }else{
      tar_axis = mat.getColumn(0);
      angle = atan2(-1*tar_axis.getZ(), tar_axis.getX());
    }
  }else if(axis == 'z' || axis == 'Z'){
    tar_axis = mat.getColumn(0);
    if(fabs(tar_axis.getZ()) < 0.9){
      angle = atan2(tar_axis.getY(), tar_axis.getX());
    }else{
      tar_axis = mat.getColumn(1);
      angle = atan2(-1*tar_axis.getX(), tar_axis.getY());
    }
  }
  assert(angle != NAN);
  return angle;
}

void angleToQuat(const double angle, tf2::Quaternion& q, char axis)
{
  geometry_msgs::msg::Quaternion q_msg;
  angleToQuat(angle, q_msg, axis);
  q = tf2::Quaternion(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  return;
}

void angleToQuat(const double angle, geometry_msgs::msg::Quaternion& q, char axis)
{
  double ang = angle;
  if(fabs(ang) > M_PI){
    ang -= std::copysign(2 * M_PI, ang);
  }
  bool axis_valid = true;
  axis_valid = axis_valid && axis_valid; // just let compiler stop report warning
  tf2::Quaternion quat(0, 0, 0, 0);
  if(axis == 'x' || axis == 'X'){
    quat.setRPY(ang, 0, 0);
  }else if(axis == 'y' || axis == 'Y'){
    quat.setRPY(0, ang, 0);
  }else if(axis == 'z' || axis == 'Z'){
    quat.setRPY(0, 0, ang);
  }else{
    axis_valid = false;
  }
  assert(axis_valid == true);
  q.w = quat.getW();
  q.x = quat.getX();
  q.y = quat.getY();
  q.z = quat.getZ();
  return;
}
// only support point ordered in x, y, rz;
std::shared_ptr<std::list<Eigen::VectorXd>>
  convertPathToPoints(const nav_msgs::msg::Path & path_msg)
{
  const auto get_curr_ang = [&](double& last_ang, double& offset, const geometry_msgs::msg::Quaternion& q){
    double curr_ang = quatToAngle(q);
    if(fabs(curr_ang - last_ang) > M_PI){
      offset -= std::copysign(2 * M_PI, curr_ang - last_ang);
    }
    last_ang = curr_ang;
    return curr_ang + offset;
  };
  std::shared_ptr<std::list<Eigen::VectorXd>> points = std::make_shared<
    std::list<Eigen::VectorXd>>();
  double running_offset = 0.0;
  double last_ang = quatToAngle(path_msg.poses[0].pose.orientation);
  for(const auto& pp : path_msg.poses)
  {
    Eigen::Vector3d p;
    const double curr_ang = get_curr_ang(last_ang, running_offset, pp.pose.orientation);
    p(0) = pp.pose.position.x;
    p(1) = pp.pose.position.y;
    p(2) = curr_ang;
    // Make sure the first pose will be push back, and if difference too small, don't add it.
    if(points->size() == 0 || std::hypot(p(0) - points->back()[0],
                                         p(1) - points->back()[1],
                                         p(2) - points->back()[2]) > mcdd::DEFAULT_TOLERANCE)
    {
      points->push_back(p);
    }
    else if(pp == path_msg.poses.back()) // Make sure the last pose include in traj
    {
      points->back() = p;
    }
  }
  return points;
}

std::shared_ptr<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>
  parameterizedToMsg(TrajectoryParameterization& parameterized, const double sample_time)
{
  std::shared_ptr<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>> points = 
    std::make_shared<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>();

  size_t sample_count = std::ceil(parameterized.getDuration() / sample_time);

  Eigen::VectorXd position, velocity, acceleration;
  points->reserve(sample_count);
  for(size_t sample = 0; sample <= sample_count; sample++)
  {
    const double t = std::min(parameterized.getDuration(), sample * sample_time);
    position = parameterized.getPosition(t);
    velocity = parameterized.getVelocity(t);
    acceleration = parameterized.getAcceleration(t);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    const Eigen::Index dof = position.size();
    assert(dof == velocity.size() && dof == acceleration.size());
    point.positions.reserve(dof);
    point.velocities.reserve(dof);
    point.accelerations.reserve(dof);
    for(Eigen::Index i = 0; i < dof; i++)
    {
      point.positions.push_back(position(i));
      point.velocities.push_back(velocity(i));
      point.accelerations.push_back(acceleration(i));
    }
    uint64_t nanosec = t * 1e9;
    point.time_from_start = rclcpp::Duration(int32_t(t), uint32_t(nanosec % uint64_t(1e9)));
    points->push_back(point);
  }
  return points;
}

bool getTf(
  const std::unique_ptr<tf2_ros::Buffer>& tf_buffer,
  const std::string& from, const std::string& to, geometry_msgs::msg::TransformStamped& t, 
  const rclcpp::Duration& time_out)
{
  // TODO(andyc): Make it stop reporting errors so frequently.
  try {
    if(!tf_buffer->canTransform(from, to, rclcpp::Time(0, 0))){
      return false;
    }
    t = tf_buffer->lookupTransform(
      from, to, tf2::TimePointZero, tf2::durationFromSec(time_out.seconds()));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      utils_logger, "Could not transform %s to %s: %s",
      from.c_str(), to.c_str(), ex.what());
    return false;
  }
  return true;
}

void msgToTf(const geometry_msgs::msg::TransformStamped& t1, tf2::Transform& t2)
{
  msgToTf(t1.transform, t2);
}

void msgToTf(const geometry_msgs::msg::Transform& t1, tf2::Transform& t2)
{
  t2.setOrigin(tf2::Vector3(t1.translation.x,
                            t1.translation.y,
                            t1.translation.z));
  t2.setRotation(tf2::Quaternion(t1.rotation.x,
                                 t1.rotation.y,
                                 t1.rotation.z,
                                 t1.rotation.w));
}

void msgToTf(const geometry_msgs::msg::PoseStamped& t1, tf2::Transform& t2)
{
  msgToTf(t1.pose, t2);
}

void msgToTf(const geometry_msgs::msg::Pose& t1, tf2::Transform& t2)
{
  t2.setOrigin(tf2::Vector3(t1.position.x,
                            t1.position.y,
                            t1.position.z));
  t2.setRotation(tf2::Quaternion(t1.orientation.x,
                                 t1.orientation.y,
                                 t1.orientation.z,
                                 t1.orientation.w));
}

void msgToTf(const geometry_msgs::msg::Quaternion& t1, tf2::Quaternion& t2)
{
  t2 = tf2::Quaternion(t1.x, t1.y, t1.z, t1.w);
}

void msgToTf(const geometry_msgs::msg::Vector3& t1, tf2::Vector3& t2)
{
  t2 = tf2::Vector3(t1.x, t1.y, t1.z);
}

void tfToMsg(const tf2::Transform& t1, geometry_msgs::msg::TransformStamped& t2)
{
  tfToMsg(t1, t2.transform);
}

void tfToMsg(const tf2::Transform& t1, geometry_msgs::msg::Transform& t2)
{
  t2.translation.x = t1.getOrigin().getX();
  t2.translation.y = t1.getOrigin().getY();
  t2.translation.z = t1.getOrigin().getZ();
  t2.rotation.x = t1.getRotation().getX();
  t2.rotation.y = t1.getRotation().getY();
  t2.rotation.z = t1.getRotation().getZ();
  t2.rotation.w = t1.getRotation().getW();
}

void tfToMsg(const tf2::Transform& t1, geometry_msgs::msg::PoseStamped& t2)
{
  tfToMsg(t1, t2.pose);
}

void tfToMsg(const tf2::Transform& t1, geometry_msgs::msg::Pose& t2)
{
  t2.position.x = t1.getOrigin().getX();
  t2.position.y = t1.getOrigin().getY();
  t2.position.z = t1.getOrigin().getZ();
  t2.orientation.x = t1.getRotation().getX();
  t2.orientation.y = t1.getRotation().getY();
  t2.orientation.z = t1.getRotation().getZ();
  t2.orientation.w = t1.getRotation().getW();
}

void tfToMsg(const tf2::Quaternion& t1, geometry_msgs::msg::Quaternion& t2)
{
  t2.x = t1.getX(); 
  t2.y = t1.getY(); 
  t2.z = t1.getZ(); 
  t2.w = t1.getW();
}

void tfToMsg(const tf2::Vector3& t1, geometry_msgs::msg::Vector3& t2)
{
  t2.x = t1.getX();
  t2.y = t1.getY();
  t2.z = t1.getZ();
}
namespace speed_limiter
{
double limit(
	double& v, double& a, const double& v_in, const double& a_in, const double& v0, 
	const double& a0, const double& dt, const mcdd::JointLimits& jl)
{
  v = v_in;
  a = a_in;
  return limit(v, a, v0, a0, dt, jl);
}
double limit(
  double& v, double& a, const double& v0, const double& a0,
  const double& dt, const mcdd::JointLimits& jl)
{
  const double tmp = v;

  limit_jerk(v, a, v0, a0, dt, jl);
  limit_acceleration(v, a, v0, dt, jl);
  limit_velocity(v, v0, jl);

  return tmp != 0.0 ? (v - v0) / (tmp - v0) : 1.0;
}

double limit_velocity(
  double& v, const double& v0, const mcdd::JointLimits& jl)
{
  const double tmp = v;

  if (jl.has_vel_limit)
  {
    v = std::clamp(v, jl.vel_min, jl.vel_max);
  }

  return tmp != 0.0 ? (v - v0) / (tmp - v0) : 1.0;
}

double limit_acceleration(
  double& v, double& a, const double& v0, const double& dt, const mcdd::JointLimits& jl)
{
  const double tmp = v;

  if (jl.has_acc_limit)
  {
    const double acc_max = (a > 0 && a < jl.acc_max) ? a : jl.acc_max;
    const double acc_min = (a < 0 && a > jl.acc_min) ? a : jl.acc_min;

    const double dv_min = acc_max * dt;
    const double dv_max = acc_min * dt;

    const double dv = v - v0;

    if(dv < dv_min){
      v = v0 + dv_min;
    }else if(dv > dv_max){
      v = v0 + dv_max;
    }
    a = dv / dt;
  }

  return tmp != 0.0 ? (v - v0) / (tmp - v0) : 1.0;
}

double limit_jerk(
  double& v, double& a, const double& v0, const double& a0, 
  const double& dt, const mcdd::JointLimits& jl)
{
  const double tmp = v;

  if (jl.has_jerk_limit)
  {
    const double dv = v - v0;
    const double dv0 = a0 * dt;

    const double dt2 = 2. * dt * dt;

    const double da_min = jl.jerk_min * dt2;
    const double da_max = jl.jerk_max * dt2;

    const double da = (fabs(dv - dv0) < fabs((a - a0))) ? dv - dv0 : a - a0;

    if(da > da_max){
      v = v0 + dv0 + da_max;
    }else if(da < da_min){
      v = v0 + dv0 + da_min;
    }
    a = da / dt;
  }

  return tmp != 0.0 ? (v - v0) / (tmp - v0) : 1.0;
}
} // end of namespace speed_limiter
} // end of namespace utils
} // end of namespace mm_controllers