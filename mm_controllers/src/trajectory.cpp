// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mm_controllers/trajectory.hpp"

#include <memory>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <limits>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rcppmath/clamp.hpp"
#include "std_msgs/msg/header.hpp"
namespace mm_controllers
{
Trajectory::Trajectory() : trajectory_start_time_(0), time_before_traj_msg_(0) {}

Trajectory::Trajectory(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{
}

Trajectory::Trajectory(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point,
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
: trajectory_msg_(joint_trajectory),
  trajectory_start_time_(static_cast<rclcpp::Time>(joint_trajectory->header.stamp))
{
  set_point_before_trajectory_msg(current_time, current_point);
  update(joint_trajectory);
}

void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;
}

void Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;
  executed_already_ = false;
}

bool Trajectory::sample(
  const rclcpp::Time & sample_time,
  const interpolation_methods::InterpolationMethod interpolation_method,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state,
  TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr)
{
  THROW_ON_NULLPTR(trajectory_msg_)
  output_state = trajectory_msgs::msg::JointTrajectoryPoint();

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  // first sampling of this trajectory
  if (!sampled_already_)
  {
    if (trajectory_start_time_.nanoseconds() <= sample_time.nanoseconds())
    {
      trajectory_start_time_ = sample_time;
    }
    if (time_before_traj_msg_.nanoseconds() <= sample_time.nanoseconds())
    {
      time_before_traj_msg_ = sample_time;
    }
    sampled_already_ = true;
  }

  // sampling before the current point
  if (sample_time.nanoseconds() < time_before_traj_msg_.nanoseconds())
  {
    return false;
  }

  auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  // current time hasn't reached traj time of the first point in the msg yet
  if (sample_time < first_point_timestamp)
  {
    // If interpolation is disabled, just forward the next waypoint
    if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
    {
      output_state = state_before_traj_msg_;
    }
    else
    {
      // it changes points only if position and velocity do not exist, but their derivatives
      deduce_from_derivatives(
        state_before_traj_msg_, first_point_in_msg, state_before_traj_msg_.positions.size(),
        (first_point_timestamp - time_before_traj_msg_).seconds());

      interpolate_between_points(
        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp, first_point_in_msg,
        sample_time, output_state);
    }
    start_segment_itr = begin();  // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of trajectory
  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      // If interpolation is disabled, just forward the next waypoint
      if (interpolation_method == interpolation_methods::InterpolationMethod::NONE)
      {
        output_state = next_point;
      }
      // Do interpolation
      else
      {
        // it changes points only if position and velocity do not exist, but their derivatives
        deduce_from_derivatives(
          point, next_point, state_before_traj_msg_.positions.size(), (t1 - t0).seconds());

        interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);
      }
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);
      return true;
    }
  }

  // whole animation has played out
  start_segment_itr = --end();
  end_segment_itr = end();
  output_state = (*start_segment_itr);
  // the trajectories in msg may have empty velocities/accel, so resize them
  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

void Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers)
  {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();
  if (duration_so_far.seconds() < 0.0)
  {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
    has_velocity = has_accel = false;
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds())
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);

  if (!has_velocity && !has_accel)
  {
    // do linear interpolation
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double end_pos = state_b.positions[i];

      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = start_pos;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  }
  else if (has_velocity && !has_accel)
  {
    // do cubic interpolation
    double T[4];
    generate_powers(3, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];

      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[2] =
          (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
        coefficients[3] =
          (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] + t[2] * 3.0 * coefficients[3];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
    }
  }
  else if (has_velocity && has_accel)
  {
    // do quintic interpolation
    double T[6];
    generate_powers(5, duration_btwn_points.seconds(), T);

    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double start_acc = state_a.accelerations[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double end_acc = state_b.accelerations[i];

      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5 * start_acc;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] +
                           end_acc * T[2] - 12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) /
                          (2.0 * T[3]);
        coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] -
                           2.0 * end_acc * T[2] + 16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) /
                          (2.0 * T[4]);
        coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
                           6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) /
                          (2.0 * T[5]);
      }

      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1] +
                            t[2] * coefficients[2] + t[3] * coefficients[3] +
                            t[4] * coefficients[4] + t[5] * coefficients[5];
      output.velocities[i] = t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
                             t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
                             t[4] * 5.0 * coefficients[5];
      output.accelerations[i] = t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
                                t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
    }
  }
}

void Trajectory::deduce_from_derivatives(
  trajectory_msgs::msg::JointTrajectoryPoint & first_state,
  trajectory_msgs::msg::JointTrajectoryPoint & second_state, 
  const size_t dim, const double delta_t)
{
  if (second_state.positions.empty())
  {
    second_state.positions.resize(dim);
    if (first_state.velocities.empty())
    {
      first_state.velocities.resize(dim, 0.0);
    }
    if (second_state.velocities.empty())
    {
      second_state.velocities.resize(dim);
      if (first_state.accelerations.empty())
      {
        first_state.accelerations.resize(dim, 0.0);
      }
      for (size_t i = 0; i < dim; ++i)
      {
        second_state.velocities[i] =
          first_state.velocities[i] +
          (first_state.accelerations[i] + second_state.accelerations[i]) * 0.5 * delta_t;
      }
    }
    for (size_t i = 0; i < dim; ++i)
    {
      // second state velocity should be reached on the end of the segment, so use middle
      second_state.positions[i] =
        first_state.positions[i] +
        (first_state.velocities[i] + second_state.velocities[i]) * 0.5 * delta_t;
    }
  }
}

TrajectoryPointConstIter Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}

rclcpp::Time Trajectory::time_from_start() const { return trajectory_start_time_; }

bool Trajectory::has_trajectory_msg() const { return trajectory_msg_.get() != nullptr; }

class LinearPathSegment : public PathSegment
{
public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : PathSegment((end - start).norm()), end_(end), start_(start)
  {
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  Eigen::VectorXd getTangent(double /* s */) const override
  {
    return (end_ - start_) / length_;
  }

  Eigen::VectorXd getCurvature(double /* s */) const override
  {
    return Eigen::VectorXd::Zero(start_.size());
  }

  std::list<double> getSwitchingPoints() const override
  {
    return std::list<double>();
  }

  LinearPathSegment* clone() const override
  {
    return new LinearPathSegment(*this);
  }

private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

class CircularPathSegment : public PathSegment
{
public:
  CircularPathSegment(
    const Eigen::VectorXd& start, const Eigen::VectorXd& intersection,
    const Eigen::VectorXd& end, double max_deviation)
  {
    if ((intersection - start).norm() < 0.000001 || (end - intersection).norm() < 0.000001)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const Eigen::VectorXd start_direction = (intersection - start).normalized();
    const Eigen::VectorXd end_direction = (end - intersection).normalized();
    const double start_dot_end = start_direction.dot(end_direction);

    // catch division by 0 in computations below
    if (start_dot_end > 0.999999 || start_dot_end < -0.999999)
    {
      length_ = 0.0;
      radius = 1.0;
      center = intersection;
      x = Eigen::VectorXd::Zero(start.size());
      y = Eigen::VectorXd::Zero(start.size());
      return;
    }

    const double angle = acos(start_dot_end);
    const double start_distance = (start - intersection).norm();
    const double end_distance = (end - intersection).norm();

    // enforce max deviation
    double distance = std::min(start_distance, end_distance);
    distance = std::min(distance, max_deviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));

    radius = distance / tan(0.5 * angle);
    length_ = angle * radius;

    center = intersection + (end_direction - start_direction).normalized() * 
      radius / cos(0.5 * angle);
    x = (intersection - distance * start_direction - center).normalized();
    y = start_direction;
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    const double angle = s / radius;
    return center + radius * (x * cos(angle) + y * sin(angle));
  }

  Eigen::VectorXd getTangent(double s) const override
  {
    const double angle = s / radius;
    return -x * sin(angle) + y * cos(angle);
  }

  Eigen::VectorXd getCurvature(double s) const override
  {
    const double angle = s / radius;
    return -1.0 / radius * (x * cos(angle) + y * sin(angle));
  }

  std::list<double> getSwitchingPoints() const override
  {
    std::list<double> switching_points;
    const double dim = x.size();
    for (unsigned int i = 0; i < dim; ++i)
    {
      double switching_angle = atan2(y[i], x[i]);
      if (switching_angle < 0.0)
      {
        switching_angle += M_PI;
      }
      const double switching_point = switching_angle * radius;
      if (switching_point < length_)
      {
        switching_points.push_back(switching_point);
      }
    }
    switching_points.sort();
    return switching_points;
  }

  CircularPathSegment* clone() const override
  {
    return new CircularPathSegment(*this);
  }

private:
  double radius;
  Eigen::VectorXd center;
  Eigen::VectorXd x;
  Eigen::VectorXd y;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double max_deviation) : length_(0.0)
{
  if (path.size() < 2)
    return;
  std::list<Eigen::VectorXd>::const_iterator path_iterator1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  while (path_iterator2 != path.end())
  {
    path_iterator3 = path_iterator2;
    ++path_iterator3;
    if (max_deviation > 0.0 && path_iterator3 != path.end())
    {
      CircularPathSegment* blend_segment =
          new CircularPathSegment(0.5 * (*path_iterator1 + *path_iterator2), *path_iterator2,
                                  0.5 * (*path_iterator2 + *path_iterator3), max_deviation);
      Eigen::VectorXd end_config = blend_segment->getConfig(0.0);
      if ((end_config - start_config).norm() > 0.000001)
      {
        path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, end_config));
      }
      path_segments_.emplace_back(blend_segment);

      start_config = blend_segment->getConfig(blend_segment->getLength());
    }
    else
    {
      path_segments_.push_back(
        std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      start_config = *path_iterator2;
    }
    path_iterator1 = path_iterator2;
    ++path_iterator2;
  }

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::unique_ptr<PathSegment>& path_segment : path_segments_)
  {
    path_segment->position_ = length_;
    std::list<double> local_switching_points = path_segment->getSwitchingPoints();
    for (std::list<double>::const_iterator point = local_switching_points.begin();
         point != local_switching_points.end(); ++point)
    {
      switching_points_.push_back(std::make_pair(length_ + *point, false));
    }
    length_ += path_segment->getLength();
    while (!switching_points_.empty() && switching_points_.back().first >= length_)
      switching_points_.pop_back();
    switching_points_.push_back(std::make_pair(length_, true));
  }
  switching_points_.pop_back();
}

Path::Path(const Path& path) : length_(path.length_), switching_points_(path.switching_points_)
{
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_)
  {
    path_segments_.emplace_back(path_segment->clone());
  }
}

double Path::getLength() const
{
  return length_;
}

PathSegment* Path::getPathSegment(double& s) const
{
  std::list<std::unique_ptr<PathSegment>>::const_iterator it = path_segments_.begin();
  std::list<std::unique_ptr<PathSegment>>::const_iterator next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_)
  {
    it = next;
    ++next;
  }
  s -= (*it)->position_;
  return (*it).get();
}

Eigen::VectorXd Path::getConfig(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const
{
  std::list<std::pair<double, bool>>::const_iterator it = switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s)
  {
    ++it;
  }
  if (it == switching_points_.end())
  {
    discontinuity = true;
    return length_;
  }
  discontinuity = it->second;
  return it->first;
}

std::list<std::pair<double, bool>> Path::getSwitchingPoints() const
{
  return switching_points_;
}

TrajectoryParameterization::TrajectoryParameterization(
  const Path& path, const Eigen::VectorXd& max_velocity, 
  const Eigen::VectorXd& max_acceleration, double time_step)
  : path_(path)
  , max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , joint_num_(max_velocity.size())
  , valid_(true)
  , time_step_(time_step)
  , cached_time_(std::numeric_limits<double>::max())
{
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
  while (valid_ && !integrateForward(trajectory_, after_acceleration) && valid_)
  {
    double before_acceleration;
    TrajectoryStep switching_point;
    if (getNextSwitchingPoint(
      trajectory_.back().path_pos_, switching_point, before_acceleration, after_acceleration))
    {
      break;
    }
    integrateBackward(
      trajectory_, switching_point.path_pos_, switching_point.path_vel_, before_acceleration);
  }

  if (valid_)
  {
    double before_acceleration = getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
    integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
  }

  if (valid_)
  {
    // Calculate timing
    std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
    std::list<TrajectoryStep>::iterator it = previous;
    it->time_ = 0.0;
    ++it;
    while (it != trajectory_.end())
    {
      it->time_ =
          previous->time_ + (it->path_pos_ - previous->path_pos_) / 
          ((it->path_vel_ + previous->path_vel_) / 2.0);
      previous = it;
      ++it;
    }
  }
}

TrajectoryParameterization::~TrajectoryParameterization()
{
}

// Returns true if end of path is reached.
bool TrajectoryParameterization::getNextSwitchingPoint(
  double path_pos, TrajectoryStep& next_switching_point,
  double& before_acceleration, double& after_acceleration)
{
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration, acceleration_after_acceleration;
  bool acceleration_reached_end;
  do
  {
    acceleration_reached_end =
        getNextAccelerationSwitchingPoint(
          acceleration_switching_point.path_pos_, acceleration_switching_point,
          acceleration_before_acceleration, acceleration_after_acceleration);
  } while (!acceleration_reached_end && acceleration_switching_point.path_vel_ > 
    getVelocityMaxPathVelocity(acceleration_switching_point.path_pos_));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration, velocity_after_acceleration;
  bool velocity_reached_end;
  do
  {
    velocity_reached_end = getNextVelocitySwitchingPoint(
      velocity_switching_point.path_pos_, velocity_switching_point,
      velocity_before_acceleration, velocity_after_acceleration);
  } while (
      !velocity_reached_end && 
      velocity_switching_point.path_pos_ <= acceleration_switching_point.path_pos_ &&
      (velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(
        velocity_switching_point.path_pos_ - EPS) ||
       velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(
        velocity_switching_point.path_pos_ + EPS)));

  if (acceleration_reached_end && velocity_reached_end)
  {
    return true;
  }
  else if (!acceleration_reached_end && (velocity_reached_end || 
           acceleration_switching_point.path_pos_ <= velocity_switching_point.path_pos_))
  {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  }
  else
  {
    next_switching_point = velocity_switching_point;
    before_acceleration = velocity_before_acceleration;
    after_acceleration = velocity_after_acceleration;
    return false;
  }
}

bool TrajectoryParameterization::getNextAccelerationSwitchingPoint(
  double path_pos, TrajectoryStep& next_switching_point,
  double& before_acceleration, double& after_acceleration)
{
  double switching_path_pos = path_pos;
  double switching_path_vel;
  while (true)
  {
    bool discontinuity;
    switching_path_pos = path_.getNextSwitchingPoint(switching_path_pos, discontinuity);

    if (switching_path_pos > path_.getLength() - EPS)
    {
      return true;
    }

    if (discontinuity)
    {
      const double before_path_vel = getAccelerationMaxPathVelocity(switching_path_pos - EPS);
      const double after_path_vel = getAccelerationMaxPathVelocity(switching_path_pos + EPS);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration = getMinMaxPathAcceleration(
        switching_path_pos - EPS, switching_path_vel, false);
      after_acceleration = getMinMaxPathAcceleration(
        switching_path_pos + EPS, switching_path_vel, true);

      if ((before_path_vel > after_path_vel ||
          getMinMaxPhaseSlope(switching_path_pos - EPS, switching_path_vel, false) >
          getAccelerationMaxPathVelocityDeriv(switching_path_pos - 2.0 * EPS)) &&
          (before_path_vel < after_path_vel || getMinMaxPhaseSlope(
            switching_path_pos + EPS, switching_path_vel, true) <
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + 2.0 * EPS)))
      {
        break;
      }
    }
    else
    {
      switching_path_vel = getAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switching_path_pos - EPS) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + EPS) > 0.0)
      {
        break;
      }
    }
  }

  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool TrajectoryParameterization::getNextVelocitySwitchingPoint(
  double path_pos, TrajectoryStep& next_switching_point,
  double& before_acceleration, double& after_acceleration)
{
  bool start = false;
  path_pos -= DEFAULT_TIMESTEP;
  do
  {
    path_pos += DEFAULT_TIMESTEP;

    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >=
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      start = true;
    }
  } while ((!start || getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
    getVelocityMaxPathVelocityDeriv(path_pos)) && path_pos < path_.getLength());

  if (path_pos >= path_.getLength())
  {
    return true;  // end of trajectory reached
  }

  double before_path_pos = path_pos - DEFAULT_TIMESTEP;
  double after_path_pos = path_pos;
  while (after_path_pos - before_path_pos > EPS)
  {
    path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      before_path_pos = path_pos;
    }
    else
    {
      after_path_pos = path_pos;
    }
  }

  before_acceleration = getMinMaxPathAcceleration(
    before_path_pos, getVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration = getMinMaxPathAcceleration(
    after_path_pos, getVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point = TrajectoryStep(
    after_path_pos, getVelocityMaxPathVelocity(after_path_pos));
  return false;
}

// Returns true if end of path is reached
bool TrajectoryParameterization::integrateForward(
  std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  std::list<std::pair<double, bool>> switching_points = path_.getSwitchingPoints();
  std::list<std::pair<double, bool>>::iterator next_discontinuity = switching_points.begin();

  while (true)
  {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos || !next_discontinuity->second))
    {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    path_vel += time_step_ * acceleration;
    path_pos += time_step_ * 0.5 * (old_path_vel + path_vel);

    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      // Avoid having a TrajectoryStep with path_pos near a switching point 
      // which will cause an almost identical. TrajectoryStep get added 
      // in the next run (https://github.com/ros-planning/moveit/issues/1665)
      if (path_pos - next_discontinuity->first < EPS)
      {
        continue;
      }
      path_vel = old_path_vel + (next_discontinuity->first - old_path_pos) * 
        (path_vel - old_path_vel) / (path_pos - old_path_pos);
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength())
    {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    }
    else if (path_vel < 0.0)
    {
      valid_ = false;
      RCLCPP_ERROR(LOGGER_TP, "Error while integrating forward: Negative path velocity");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos, getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos))
    {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) || 
      path_vel > getVelocityMaxPathVelocity(path_pos))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > EPS)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));

      if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
      {
        if (after > next_discontinuity->first)
        {
          return false;
        }
        else if (getMinMaxPhaseSlope(
          trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
          getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
      else
      {
        if (getMinMaxPhaseSlope(
          trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
          getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

void TrajectoryParameterization::integrateBackward(
  std::list<TrajectoryStep>& start_trajectory, double path_pos, 
  double path_vel, double acceleration)
{
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  --start2;
  std::list<TrajectoryStep>::iterator start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope = 0;
  assert(start1->path_pos_ <= path_pos);

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    if (start1->path_pos_ <= path_pos)
    {
      trajectory.push_front(TrajectoryStep(path_pos, path_vel));
      path_vel -= time_step_ * acceleration;
      path_pos -= time_step_ * 0.5 * (path_vel + trajectory.front().path_vel_);
      acceleration = getMinMaxPathAcceleration(path_pos, path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) / 
        (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0)
      {
        valid_ = false;
        RCLCPP_ERROR(LOGGER_TP, 
          "Error while integrating backward: Negative path velocity");
        end_trajectory_ = trajectory;
        return;
      }
    }
    else
    {
      --start1;
      --start2;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double start_slope = 
      (start2->path_vel_ - start1->path_vel_) / (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos - 
        start_slope * start1->path_pos_) / (slope - start_slope);
    if (std::max(start1->path_pos_, path_pos) - EPS <= intersection_path_pos &&
        intersection_path_pos <= EPS + std::min(start2->path_pos_, trajectory.front().path_pos_))
    {
      const double intersection_path_vel =
          start1->path_vel_ + start_slope * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  RCLCPP_ERROR(LOGGER_TP, "Error while integrating backward: Did not hit start trajectory");
  end_trajectory_ = trajectory;
}

double TrajectoryParameterization::getMinMaxPathAcceleration(
  double path_pos, double path_vel, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      max_path_acceleration =
          std::min(max_path_acceleration,
            max_acceleration_[i] / std::abs(config_deriv[i]) - 
            factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i]);
    }
  }
  return factor * max_path_acceleration;
}

double TrajectoryParameterization::getMinMaxPhaseSlope(
  double path_pos, double path_vel, bool max)
{
  return getMinMaxPathAcceleration(path_pos, path_vel, max) / path_vel;
}

double TrajectoryParameterization::getAccelerationMaxPathVelocity(double path_pos) const
{
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < joint_num_; ++j)
      {
        if (config_deriv[j] != 0.0)
        {
          double a_ij = config_deriv2[i] / config_deriv[i] - config_deriv2[j] / config_deriv[j];
          if (a_ij != 0.0)
          {
            max_path_velocity = std::min(
              max_path_velocity, sqrt((max_acceleration_[i] / std::abs(config_deriv[i]) +
                                       max_acceleration_[j] / std::abs(config_deriv[j])) /
                                       std::abs(a_ij)));
          }
        }
      }
    }
    else if (config_deriv2[i] != 0.0)
    {
      max_path_velocity = std::min(
        max_path_velocity, sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}

double TrajectoryParameterization::getVelocityMaxPathVelocity(double path_pos) const
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    max_path_velocity = std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double TrajectoryParameterization::getAccelerationMaxPathVelocityDeriv(double path_pos)
{
  return (getAccelerationMaxPathVelocity(path_pos + EPS) - 
          getAccelerationMaxPathVelocity(path_pos - EPS)) /
         (2.0 * EPS);
}

double TrajectoryParameterization::getVelocityMaxPathVelocityDeriv(double path_pos)
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint;
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    const double this_max_path_velocity = max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity)
    {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint] * 
    path_.getCurvature(path_pos)[active_constraint]) /
    (tangent[active_constraint] * std::abs(tangent[active_constraint]));
}

bool TrajectoryParameterization::isValid() const
{
  return valid_;
}

double TrajectoryParameterization::getDuration() const
{
  return trajectory_.back().time_;
}

std::list<TrajectoryParameterization::TrajectoryStep>::const_iterator 
  TrajectoryParameterization::getTrajectorySegment(double time) const
{
  if (time >= trajectory_.back().time_)
  {
    std::list<TrajectoryStep>::const_iterator last = trajectory_.end();
    last--;
    return last;
  }
  else
  {
    if (time < cached_time_)
    {
      cached_trajectory_segment_ = trajectory_.begin();
    }
    while (time >= cached_trajectory_segment_->time_)
    {
      ++cached_trajectory_segment_;
    }
    cached_time_ = time;
    return cached_trajectory_segment_;
  }
}

Eigen::VectorXd TrajectoryParameterization::getPosition(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - 
      time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 
      0.5 * time_step * time_step * acceleration;

  return path_.getConfig(path_pos);
}

Eigen::VectorXd TrajectoryParameterization::getVelocity(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - 
      time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 
      0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;

  return path_.getTangent(path_pos) * path_vel;
}

Eigen::VectorXd TrajectoryParameterization::getAcceleration(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - 
      time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 
      0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;
  Eigen::VectorXd path_acc =
      (path_.getTangent(path_pos) * path_vel - 
      path_.getTangent(previous->path_pos_) * previous->path_vel_);
  if (time_step > 0.0)
    path_acc /= time_step;
  return path_acc;
}

}  // namespace mm_controllers
