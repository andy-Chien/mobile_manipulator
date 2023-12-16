#include <cstdio>
#include <iostream>
#include <cmath>
#include <numeric>
#include "mm_controllers/four_wheel_steering_kinematics.hpp"
namespace mm_controllers
{
FourWheelSteeringKinematics::FourWheelSteeringKinematics()
{
}

FourWheelSteeringKinematics::~FourWheelSteeringKinematics()
{
}

void FourWheelSteeringKinematics::configuration(mcdd::BaseData &base_data)
{
  size_t num_wheels = base_data.wheel_data_vector.size();
  mat_a_.resize(num_wheels * 2, 3);
  mat_vehicle_.resize(3, 1);
  mat_wheels_.resize(num_wheels * 2, 1);
  for(Eigen::Index i = 0; i < mat_a_.rows(); i++)
  {
    mat_a_(i, 0) = (i + 1) % 2;
    mat_a_(i, 1) = i % 2;
    mat_a_(i, 2) = ((i % 2 == 0)? -1 : 1) * base_data.wheel_data_vector[
      size_t(i / 2)]->pos_on_vehicle[(i + 1) % 2];
  }
}

bool FourWheelSteeringKinematics::checkSlippage(mcdd::BaseData &base_data)
{
  double max_vbw = 0;
  bool has_slippage = false;
  std::string max_vbw_wheel;
  Eigen::Vector2d v_1, v_2, l_ij;
  for(auto it_fst = base_data.wheel_data.begin(); 
    it_fst != base_data.wheel_data.end(); it_fst++)
  {
    double vbw = 0;
    it_fst->second->has_slippage = false;
    for(auto it_sec = base_data.wheel_data.begin(); 
      it_sec != base_data.wheel_data.end(); it_sec++)
    {
      if(it_fst == it_sec)
        continue;
      v_1 << it_fst->second->curr_state.direction[0] * it_fst->second->curr_state.velocity,
           it_fst->second->curr_state.direction[1] * it_fst->second->curr_state.velocity;
      v_2 << it_sec->second->curr_state.direction[0] * it_fst->second->curr_state.velocity,
           it_sec->second->curr_state.direction[1] * it_fst->second->curr_state.velocity;
      l_ij << it_fst->second->pos_on_vehicle[0] - it_sec->second->pos_on_vehicle[0],
          it_fst->second->pos_on_vehicle[1] - it_sec->second->pos_on_vehicle[1];
      l_ij /= l_ij.norm();
      vbw += fabs((v_1 - v_2).dot(l_ij));
    }
    if(vbw > max_vbw)
    {
      max_vbw = vbw;
      has_slippage = true;
      max_vbw_wheel = it_fst->first;
    }
  }
  if(has_slippage)
  {
    base_data.wheel_data[max_vbw_wheel]->has_slippage = true;
  }
  return true;
}

bool FourWheelSteeringKinematics::forwardKinematics(mcdd::BaseData &base_data)
{
  if(!checkSlippage(base_data))
  {
    return false;
  }

  double vel;
  Eigen::MatrixXd mat_a(mat_a_);
  Eigen::MatrixXd mat_wheels(mat_wheels_);
  for(size_t i = 0; i < base_data.wheel_data_vector.size(); i++)
  {
    vel = base_data.wheel_data_vector[i]->curr_state.velocity;
    mat_wheels(i * 2, 0) = base_data.wheel_data_vector[i]->curr_state.direction[0] * vel;
    mat_wheels(i * 2 + 1, 0) = base_data.wheel_data_vector[i]->curr_state.direction[1] * vel;
  }
  for(size_t i = 0; i < base_data.wheel_data_vector.size(); i++)
  {
    if(base_data.wheel_data_vector[i]->has_slippage)
    {
      matrixRemoveRow(mat_a, i * 2);
      matrixRemoveRow(mat_a, i * 2 + 1);
      matrixRemoveRow(mat_wheels, i * 2);
      matrixRemoveRow(mat_wheels, i * 2 + 1);
    }
  }

  Eigen::MatrixXd mat_vehicle = 
    (mat_a.transpose() * mat_a).ldlt().solve(mat_a.transpose() * mat_wheels);
  vel = std::hypot(mat_vehicle(0, 0), mat_vehicle(1, 0));
  base_data.curr_state.lin_vel = vel;
  base_data.curr_state.ang_vel = mat_vehicle(2, 0);
  if(vel > 0.0001)
  {
    base_data.curr_state.direction[0] = mat_vehicle(0, 0) / vel;
    base_data.curr_state.direction[1] = mat_vehicle(1, 0) / vel;
  }
  compute_icr(base_data.curr_state, base_data.curr_state);
  return true;
}

bool FourWheelSteeringKinematics::inverseKinematics(
  mcdd::BaseData &base_data, bool for_goal)
{
  double vel;
  mcdd::BaseState& bs = (for_goal) ? base_data.goal_state : base_data.next_state;
  std::vector<mcdd::WheelData*> wv = base_data.wheel_data_vector;
  mat_vehicle_(0, 0) = bs.direction[0] * bs.lin_vel;
  mat_vehicle_(1, 0) = bs.direction[1] * bs.lin_vel;
  mat_vehicle_(2, 0) = bs.ang_vel;
  mat_wheels_ = mat_a_ * mat_vehicle_;
  for(size_t i = 0; i < wv.size(); i++)
  {
    mcdd::WheelState& ws = (for_goal) ? wv[i]->goal_state : wv[i]->next_state;
    vel = std::hypot(mat_wheels_(i * 2, 0), mat_wheels_(i * 2 + 1, 0));
    ws.velocity = vel;
    if(vel > 0.0001)
    {
      ws.direction[0] = mat_wheels_(i * 2, 0) / vel;
      ws.direction[1] = mat_wheels_(i * 2 + 1, 0) / vel;
    }
    else
    {
      ws.direction[0] = wv[i]->curr_state.direction[0];
      ws.direction[1] = wv[i]->curr_state.direction[1];
    }
  }
  compute_icr(bs, bs);
  return true;
}

bool FourWheelSteeringKinematics::cmdForwardKinematics(mcdd::BaseData &base_data)
{
  if(!checkSlippage(base_data))
  {
    return false;
  }
  double vel;
  Eigen::MatrixXd mat_a(mat_a_);
  Eigen::MatrixXd mat_wheels(mat_wheels_);
  for(size_t i = 0; i < base_data.wheel_data_vector.size(); i++)
  {
    vel = base_data.wheel_data_vector[i]->curr_state.velocity;
    mat_wheels(i * 2, 0) = base_data.wheel_data_vector[i]->curr_state.direction[0] * vel;
    mat_wheels(i * 2 + 1, 0) = base_data.wheel_data_vector[i]->curr_state.direction[1] * vel;
  }
  for(size_t i = 0; i < base_data.wheel_data_vector.size(); i++)
  {
    if(base_data.wheel_data_vector[i]->has_slippage)
    {
      matrixRemoveRow(mat_a, i * 2);
      matrixRemoveRow(mat_a, i * 2 + 1);
      matrixRemoveRow(mat_wheels, i * 2);
      matrixRemoveRow(mat_wheels, i * 2 + 1);
    }
  }

  Eigen::MatrixXd mat_vehicle = (
    mat_a.transpose() * mat_a).ldlt().solve(mat_a.transpose() * mat_wheels);
  vel = std::hypot(mat_vehicle(0, 0), mat_vehicle(1, 0));
  base_data.next_state.lin_vel = vel;
  base_data.next_state.ang_vel = mat_vehicle(2, 0);
  if(vel > 0.0001)
  {
    base_data.next_state.direction[0] = mat_vehicle(0, 0) / vel;
    base_data.next_state.direction[1] = mat_vehicle(1, 0) / vel;
  }
  else
  {
    base_data.next_state.direction[0] = base_data.curr_state.direction[0];
    base_data.next_state.direction[1] = base_data.curr_state.direction[1];
  }
  compute_icr(base_data.next_state, base_data.curr_state);
  return true;
}

void FourWheelSteeringKinematics::angularVelocityToDirection(
  const double &ang_vel, const mcdd::Double2 &pos_on_vehicle, mcdd::Double2 &direction)
{
  // position include the radius of rotation, so no need to calculate radius.
  // when ang_vel is positive, direction is exchange from position of wheel,
  // and trans fist item to negative, when ang_vel is negative, formula is the same.
  direction[0] = -1 * ang_vel * pos_on_vehicle[1];
  direction[1] = ang_vel * pos_on_vehicle[0];
}

void FourWheelSteeringKinematics::matrixRemoveRow(
  Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
  unsigned int numRows = matrix.rows()-1;
  unsigned int numCols = matrix.cols();

  if(rowToRemove < numRows){
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = 
      matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);
  }
  matrix.conservativeResize(numRows, numCols);
}

bool FourWheelSteeringKinematics::wheeledIK(
  mcdd::WheelData* wheel_data, bool for_goal)
{
  wheel_data->want_to_stop = false;
  wheel_data->have_to_stop = false;
  mcdd::WheelState& ws = (for_goal) ? wheel_data->goal_state : wheel_data->next_state;
  const double& wheel_radius = wheel_data->radius;
  const double& s_pos = wheel_data->curr_state.st_pos;
  const double& w_vel = wheel_data->curr_state.dr_vel;
  const mcdd::Double2& dir_goal = wheel_data->goal_state.direction;
  const mcdd::Double2& direction = wheel_data->curr_state.direction;
  const mcdd::Double2& dir_tar = ws.direction;
  const mcdd::JointLimits& s_j_limits = wheel_data->swerve_joint_limits;
  const mcdd::JointLimits& w_j_limits = wheel_data->wheel_joint_limits;
  double s_ang_tar, w_vel_tar, s_ang_goal, s_diff;
  double s_ang = atan2(direction[1], direction[0]);

  s_ang_goal = atan2(dir_goal[1], dir_goal[0]);
  s_ang_tar = atan2(dir_tar[1], dir_tar[0]);
  w_vel_tar = metersToRads(ws.velocity, wheel_radius);

  if(wheel_data->is_halted)
  {
    if(fabs(s_ang_goal - s_ang) > M_PI)
    {
      s_ang -= std::copysign(2 * M_PI, s_ang);
    }
    if(fabs(s_ang_goal - s_ang) > M_PI_2)
    {
      s_ang -= std::copysign(M_PI, s_ang);
    }
  }

  s_diff = s_ang_tar - s_ang;

  if(fabs(s_diff) > M_PI)
  {
    s_diff -= std::copysign(2 * M_PI, s_diff);
  }

  if(for_goal && fabs(s_diff) > M_PI_2)
  {
    if(s_j_limits.has_vel_limit && w_j_limits.has_acc_limit)
    {
      double time_fwd, time_rvs, time_s, time_w;
      double w_vel_diff_fwd, s_ang_diff_fwd, w_vel_diff_rvs, s_ang_diff_rvs;
      const mcdd::Double2& dir_move = {dir_tar[0] * w_vel_tar - direction[0], 
                     dir_tar[1] * w_vel_tar - direction[1]};
      double a = fabs(w_vel);
      double b = w_vel_tar;
      double c = std::hypot(dir_move[0], dir_move[1]);
      double s = (a + b + c) / 2;
      double vel_min = 2 * sqrt(s * (s-a) * (s-b) * (s-c)) / c;
      s_ang_diff_fwd = fabs(s_diff);
      w_vel_diff_fwd = fabs(w_vel_tar - vel_min) + fabs(vel_min - fabs(w_vel));
      s_ang_diff_rvs = fabs(s_diff - std::copysign(M_PI, s_diff));
      w_vel_diff_rvs = w_vel_tar + fabs(w_vel);
      if(s_j_limits.has_acc_limit)
      {
        if(s_ang_diff_fwd <= pow(s_j_limits.vel, 2) / s_j_limits.acc)
        {
          time_s = sqrt(s_ang_diff_fwd / s_j_limits.acc);
        }
        else
        {
          time_s = s_ang_diff_fwd / s_j_limits.vel + s_j_limits.vel / s_j_limits.acc;
        }
        time_w = w_vel_diff_fwd / w_j_limits.acc;
        time_fwd = time_s + time_w;
        if(s_ang_diff_rvs <= pow(s_j_limits.vel, 2) / s_j_limits.acc)
        {
          time_s = sqrt(s_ang_diff_rvs / s_j_limits.acc);
        }
        else
        {
          time_s = s_ang_diff_rvs / s_j_limits.vel + s_j_limits.vel / s_j_limits.acc;
        }
        time_w = w_vel_diff_rvs / w_j_limits.acc;
        time_rvs = time_s + time_w;
      }
      else
      {
        time_s = s_ang_diff_fwd / s_j_limits.vel;
        time_w = w_vel_diff_fwd / w_j_limits.acc;
        time_fwd = time_s + time_w;

        time_s = s_ang_diff_rvs / s_j_limits.vel;
        time_w = w_vel_diff_rvs / w_j_limits.acc;
        time_rvs = time_s + time_w;
      }
      if(time_rvs < time_fwd)
      {
        wheel_data->want_to_stop = true;
      }
    }
    else
    {
      wheel_data->want_to_stop = true;
    }
  }
  else if(fabs(s_diff) > M_PI_2)
  {
    s_diff -= std::copysign(M_PI, s_diff);
  }

  double diss = fabs(std::fmod(s_ang_tar - s_pos, 2 * M_PI));

  ws.st_pos = s_pos + s_diff;
  ws.dr_vel = (M_PI_2 < diss && diss < M_PI_2 * 3) ? -1 * w_vel_tar : w_vel_tar;

  if(s_j_limits.has_pos_limit)
  {
    const double& p_max = 
      (s_j_limits.pos_max > s_j_limits.pos_min) ? s_j_limits.pos_max : s_j_limits.pos_min;
    const double& p_min = 
      (s_j_limits.pos_max < s_j_limits.pos_min) ? s_j_limits.pos_max : s_j_limits.pos_min;
    const double& soft_p_max = p_max - M_PI_4;
    const double& soft_p_min = p_min + M_PI_4;
    if(for_goal && wheel_data->is_halted && 
      (ws.st_pos > soft_p_max || ws.st_pos < soft_p_min))
    {
      wheel_data->want_to_stop = true;
      if(for_goal && soft_p_max - soft_p_min > M_PI){
        ws.st_pos -= std::copysign(M_PI, ws.st_pos - soft_p_max);
      }
    }
    else if(ws.st_pos > p_max || ws.st_pos < p_min)
    {
      wheel_data->have_to_stop = true;
      if(for_goal && p_max - p_min > M_PI){
        ws.st_pos -= std::copysign(M_PI, ws.st_pos - p_max);
      }
    }
  }
  return true;
}

void FourWheelSteeringKinematics::compute_icr(
  mcdd::BaseState& base_state, const mcdd::BaseState& ref_state)
{
  const double ref_ang_vel = (fabs(base_state.ang_vel) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT * 0.001) ? 
    base_state.ang_vel : mcdd::DEFAULT_SMALL_ENOUGH_FLOAT * 0.001;
  if(fabs(base_state.lin_vel) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT &&
     fabs(base_state.ang_vel) > mcdd::DEFAULT_SMALL_ENOUGH_FLOAT)
  {
    base_state.icr_pos[0] = -1 * base_state.direction[1] * base_state.lin_vel / ref_ang_vel;
    base_state.icr_pos[1] = base_state.direction[0] * base_state.lin_vel / ref_ang_vel;
  
    const double icr_length = std::hypot(base_state.icr_pos[0], base_state.icr_pos[1]);
    if(icr_length > mcdd::DEFAULT_MAX_ICR_LENGTH)
    {
      base_state.icr_pos[0] *= mcdd::DEFAULT_MAX_ICR_LENGTH / icr_length;
      base_state.icr_pos[1] *= mcdd::DEFAULT_MAX_ICR_LENGTH / icr_length;
    }
    base_state.icr_rot = atan2(base_state.icr_pos[1], base_state.icr_pos[0]);
  }
  else
  {
    base_state.icr_pos[0] = ref_state.icr_pos[0];
    base_state.icr_pos[1] = ref_state.icr_pos[1];
    base_state.icr_rot = ref_state.icr_rot;
  }
}

bool FourWheelSteeringKinematics::checkSingularity(mcdd::BaseData& base_data, bool for_goal)
{
  const double rsz = 0.1; // radius of singularity zone
  mcdd::BaseState& bs = (for_goal) ? base_data.goal_state : base_data.next_state;
  auto& dt = bs.direction;
  auto& at = bs.ang_vel;
  const auto& last_dt = base_data.last_state.direction;
  const auto& last_at = base_data.last_state.ang_vel;
  double& dt_length = bs.lin_vel;
  double last_dt_length = base_data.last_state.lin_vel;
  double abs_at = fabs(at);
  double abs_last_at = fabs(last_at);
  if(abs_at < 0.0001 || dt_length < 0.0001){
    return true;
  }
  if(!for_goal && (abs_last_at < 0.0001 || last_dt_length < 0.0001)){
    return true;
  }
  
  mcdd::Double2 rc = {-1 * dt[1] * dt_length / at, dt[0] * dt_length / at}; // rotation center
  mcdd::Double2 last_rc = {-1 * last_dt[1] * last_dt_length / last_at, 
                                last_dt[0] * last_dt_length / last_at};
  std::cout<<"checkSingularity ";
  std::cout<<"dt = ("<<dt[0]<<", "<<dt[1]<<")";
  for(auto& wheel_data : base_data.wheel_data_vector)
  {
    const auto& pv = wheel_data->pos_on_vehicle;
    mcdd::Double2 pv_rc = {rc[0] - pv[0], rc[1] - pv[1]};
    double pv_rc_length = std::hypot(pv_rc[0], pv_rc[1]);
    if(pv_rc_length >= rsz){
      continue;
    }
    if(for_goal)
    {
      pv_rc[0] *= rsz / pv_rc_length;
      pv_rc[1] *= rsz / pv_rc_length;
      double rc_l = std::hypot(rc[0], rc[1]);
      rc[0] = pv[0] + pv_rc[0];
      rc[1] = pv[1] + pv_rc[1];
      double rc_new_l = std::hypot(rc[0], rc[1]);
      double factor = (rc_new_l - rc_l) / (rc_new_l + rc_l);
      at *= (1 - factor);
      mcdd::Double2 dt_new = {rc[1] * at, rc[0] * at * -1};
      dt_length = hypot(dt_new[0], dt_new[1]);
      dt[0] = dt_new[0] / dt_length;
      dt[1] = dt_new[1] / dt_length;
    }
    else
    {
      std::cout<<"1, ";
      mcdd::Double2 last_rc_rc = {rc[0] * abs_at - last_rc[0] * abs_at, 
                                  rc[1] * abs_at - last_rc[1] * abs_at};
      mcdd::Double2 last_rc_pv = {pv[0] * abs_at - last_rc[0] * abs_at, 
                                  pv[1] * abs_at - last_rc[1] * abs_at};
      double a = std::hypot(last_rc_rc[0], last_rc_rc[1]);
      double b = std::hypot(last_rc_pv[0], last_rc_pv[1]);
      double c = rsz * abs_at;
      std::cout<<"2, ";
      double theta;
      if(a * a + b * b > c * c){
        theta = acos((a * a + b * b - c * c) / (2 * a * b));
      }else{
        theta = M_PI_2;
      }
      std::cout<<"3, ";
      mcdd::Double2 pos_1 = 
        {(cos(theta) * last_rc_pv[0] - sin(theta) * last_rc_pv[1]) * a / b,
         (sin(theta) * last_rc_pv[0] + cos(theta) * last_rc_pv[1]) * a / b};
      mcdd::Double2 pos_2 = 
        {(cos(-1 * theta) * last_rc_pv[0] - sin(-1 * theta) * last_rc_pv[1]) * a / b,
         (sin(-1 * theta) * last_rc_pv[0] + cos(-1 * theta) * last_rc_pv[1]) * a / b};
      std::cout<<"4, ";
      double theta_1 = (pos_1[0] * last_rc_rc[0] + pos_1[1] * last_rc_rc[1]) / (a * a);
      double theta_2 = (pos_2[0] * last_rc_rc[0] + pos_2[1] * last_rc_rc[1]) / (a * a);
      std::cout<<"5, ";
      if(theta_1 < theta_2)
      {
        rc[0] = pos_1[0] / abs_at + last_rc[0];
        rc[1] = pos_1[1] / abs_at + last_rc[1];
      }
      else if(theta_2 < theta_1)
      {
        rc[0] = pos_2[0] / abs_at + last_rc[0];
        rc[1] = pos_2[1] / abs_at + last_rc[1];
      }
      else if(pos_1[0] * last_rc[0] + pos_1[1] * last_rc[1] <= 0)
      {
        rc[0] = pos_1[0] / abs_at + last_rc[0];
        rc[1] = pos_1[1] / abs_at + last_rc[1];
      }
      else
      {
        rc[0] = pos_2[0] / abs_at + last_rc[0];
        rc[1] = pos_2[1] / abs_at + last_rc[1];
      }
      std::cout<<"6, ";
      mcdd::Double2 dt_new = {rc[1] * at, rc[0] * at * -1};
      dt_length = hypot(dt_new[0], dt_new[1]);
      dt[0] = dt_new[0] / dt_length;
      dt[1] = dt_new[1] / dt_length;
      std::cout<<"dt = ("<<dt[0]<<", "<<dt[1]<<"), ";
      std::cout<<"a = "<<a<<", b = "<<b<<", c = "<<c<<", theta = "<<theta;
    }
    std::cout<<std::endl;
    return false;
  }
  return true;
}

double FourWheelSteeringKinematics::metersToRads(
  const double& meters, const double& radius)
{
  return meters / radius;
}
}