#include "rclcpp/rclcpp.hpp"
#include "mm_msgs/srv/path_to_traj.hpp"
#include "mm_msgs/srv/traj_merge.hpp"
#include "mm_controllers/trajectory.hpp"
#include "mm_controllers/interpolation_methods.hpp"
#include "mm_controllers/mm_controller_utils.hpp"

#include <memory>

namespace mcu = mm_controllers::utils;

class PathToTrajectoryServer : public rclcpp::Node
{
  public:
    PathToTrajectoryServer()
    : Node("path_to_trajectory_server")
    {
      path_to_traj_service_ = this->create_service<mm_msgs::srv::PathToTraj>(
        "path_to_trajectory", std::bind(
        &PathToTrajectoryServer::convert_cb, this, std::placeholders::_1, std::placeholders::_2));
      traj_merging_service_ = this->create_service<mm_msgs::srv::TrajMerge>(
        "trajectory_merging", std::bind(
        &PathToTrajectoryServer::merging_cb, this, std::placeholders::_1, std::placeholders::_2));
      param_client_node_ = rclcpp::Node::make_shared("param_client_node");
      param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        param_client_node_, "mm_trajectory_controller");

      RCLCPP_INFO(this->get_logger(), "Ready to convert path to trajectorie.");
    }
  private:
    void convert_cb(const std::shared_ptr<mm_msgs::srv::PathToTraj::Request> req,
      std::shared_ptr<mm_msgs::srv::PathToTraj::Response> res)
    {
      if(!param_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_ERROR(
          this->get_logger(), "Unable to get connect to controller manager.");
        return;
      }

      Eigen::Vector3d max_vel, max_acc;
      const bool has_params = [&]{ return
        param_client_->has_parameter("base_params.linear_vel") &&
        param_client_->has_parameter("base_params.angular_vel") &&
        param_client_->has_parameter("base_params.linear_acc") &&
        param_client_->has_parameter("base_params.angular_acc");
      }();

      if(!has_params){
        RCLCPP_ERROR(
          this->get_logger(), "Unable to get base parameters.");
        return;
      }

      const double dv = param_client_->get_parameter<double>("base_params.linear_vel") / 2;
      const double da = param_client_->get_parameter<double>("base_params.linear_acc") / 2;

      max_vel<<dv, dv, param_client_->get_parameter<double>("base_params.angular_vel") / 2;
      max_acc<<da, da, param_client_->get_parameter<double>("base_params.angular_acc") / 2;
      if(req->path.poses.empty()){
        RCLCPP_ERROR(
        this->get_logger(), "Can't convert path to trajectory, path is empty.");
        return;
      }

      std::shared_ptr<std::list<Eigen::VectorXd>> points = 
        mcu::convertPathToPoints(req->path);

      res->trajectory.header.frame_id = req->path.header.frame_id;
      res->trajectory.joint_names.push_back("linear_x");
      res->trajectory.joint_names.push_back("linear_y");
      res->trajectory.joint_names.push_back("angular_z");

      if(points->size() == 1){
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(points->front()[0]);
        point.positions.push_back(points->front()[1]);
        point.positions.push_back(points->front()[2]);
        point.time_from_start = rclcpp::Duration(0, 0);
        res->trajectory.points.push_back(point);
        return;
      }

      const double sample_time = 0.1;

      mm_controllers::TrajectoryParameterization parameterized(
        mm_controllers::Path(*points, sample_time), max_vel, max_acc, mm_controllers::DEFAULT_TIMESTEP);

      if (!parameterized.isValid())
      {
        RCLCPP_ERROR(
          this->get_logger(), "Unable to parameterize trajectory.");
        return;
      }

      res->trajectory.points = *(mcu::parameterizedToMsg(parameterized, sample_time));
    }
    void merging_cb(const std::shared_ptr<mm_msgs::srv::TrajMerge::Request> req,
      std::shared_ptr<mm_msgs::srv::TrajMerge::Response> res)
    {
      for(const trajectory_msgs::msg::JointTrajectory& traj_msg : req->trajectories)
      {
        if(!(traj_msg.points.size() > 1)){
          RCLCPP_ERROR(
            this->get_logger(), "Trajectory size <= 1.");
          res->success = false;
          return;
        }
      }
      const auto [sample_time, running_time] = [&]{
        uint64_t max_time(0), min_time(1e8), t;
        for(const trajectory_msgs::msg::JointTrajectory& traj_msg : req->trajectories)
        {
          t = rclcpp::Duration(traj_msg.points.back().time_from_start).nanoseconds();
          if(t / (traj_msg.points.size() - 1) < min_time){
            min_time = t / (traj_msg.points.size() - 1);
          }
          if(t > max_time){
            max_time = t;
          }
        }
        min_time = (min_time / 1e6) * 1e6;
        return std::tuple<double, double>(
          {double(min_time) / 1e9, double(max_time) / 1e9});
      }();

      res->trajectory.header = req->trajectories[0].header;

      std::vector<std::shared_ptr<mm_controllers::Trajectory>> trajectories;
      trajectories.reserve(req->trajectories.size());
      for(trajectory_msgs::msg::JointTrajectory& traj_msg : req->trajectories)
      {
        res->trajectory.joint_names.insert(res->trajectory.joint_names.end(), 
          traj_msg.joint_names.begin(), traj_msg.joint_names.end());

        const double time_scale = running_time / 
          rclcpp::Duration(traj_msg.points.back().time_from_start).seconds();
        if(time_scale > 1.0)
        {
          const bool update_vel = 
            traj_msg.points[0].velocities.size() == traj_msg.joint_names.size();
          const bool update_acc = 
            traj_msg.points[0].accelerations.size() == traj_msg.joint_names.size();
          const bool update_eff = 
            traj_msg.points[0].effort.size() == traj_msg.joint_names.size();
          for(trajectory_msgs::msg::JointTrajectoryPoint& point : traj_msg.points)
          {
            point.time_from_start = rclcpp::Duration(point.time_from_start) * time_scale;
            for(size_t i=0; i < traj_msg.joint_names.size(); i++)
            {
              if(update_vel){
                point.velocities[i] /= time_scale;
              }
              if(update_acc){
                point.accelerations[i] /= time_scale;
              }
              if(update_eff){
                point.effort[i] /= time_scale;
              }
            }
          }
        }
        std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg_ptr = 
          std::make_shared<trajectory_msgs::msg::JointTrajectory>(traj_msg);
        traj_msg_ptr->header.stamp = rclcpp::Time(0);

        std::shared_ptr<mm_controllers::Trajectory> traj = 
          std::make_shared<mm_controllers::Trajectory>(traj_msg_ptr);
        trajectories.push_back(traj);
      }

      const size_t sample_count = std::ceil(running_time / sample_time);
      trajectory_msgs::msg::JointTrajectoryPoint state_desired, state_merged;
      mm_controllers::interpolation_methods::InterpolationMethod interpolation_method = 
        mm_controllers::interpolation_methods::DEFAULT_INTERPOLATION;

      for(size_t sample = 0; sample <= sample_count; sample++)
      {
        const rclcpp::Time t(uint64_t(sample*sample_time * 1e9), RCL_ROS_TIME);
        state_merged = trajectory_msgs::msg::JointTrajectoryPoint();
        for(std::shared_ptr<mm_controllers::Trajectory>& traj : trajectories)
        {
          mm_controllers::TrajectoryPointConstIter start_itr, end_itr;
          const bool valid_point = 
            traj->sample(
              t, interpolation_method, state_desired, start_itr, end_itr);
          if(valid_point)
          {
            state_merged.positions.insert(state_merged.positions.end(),
              state_desired.positions.begin(), state_desired.positions.end());
            state_merged.velocities.insert(state_merged.velocities.end(),
              state_desired.velocities.begin(), state_desired.velocities.end());
            state_merged.accelerations.insert(state_merged.accelerations.end(),
              state_desired.accelerations.begin(), state_desired.accelerations.end());
          }
          else
          {
            RCLCPP_ERROR(
              this->get_logger(), "Trajectory sample failed.");
            res->success = false;
            return;
          }
        }
        const uint64_t nanosec = t.nanoseconds();
        state_merged.time_from_start = rclcpp::Duration(
          int32_t(nanosec / 1e9), uint32_t(nanosec % uint64_t(1e9)));
        res->trajectory.points.push_back(state_merged);
      }
      res->success = true;
      return;
    }
    rclcpp::Service<mm_msgs::srv::PathToTraj>::SharedPtr path_to_traj_service_;
    rclcpp::Service<mm_msgs::srv::TrajMerge>::SharedPtr traj_merging_service_;
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    std::shared_ptr<rclcpp::Node> param_client_node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathToTrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}