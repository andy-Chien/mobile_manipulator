#pragma once
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "mm_controllers/mm_data_define.hpp"

namespace mm_controllers
{
namespace mcdd = mm_controllers::data_define;
class FourWheelSteeringKinematics
{
public:
  FourWheelSteeringKinematics();
  ~FourWheelSteeringKinematics();
  void configuration(
    mcdd::BaseData& base_data);
  bool forwardKinematics(
    mcdd::BaseData& base_data);
  bool cmdForwardKinematics(
    mcdd::BaseData& base_data);
  bool inverseKinematics(
    mcdd::BaseData& base_data, bool for_goal);
  bool wheeledIK(
    mcdd::WheelData* wheel_data, bool for_goal);
  void compute_icr(
    mcdd::BaseState& base_state, const mcdd::BaseState& ref_state); 
  bool checkSingularity(
    mcdd::BaseData& base_data, bool for_goal);
private:
  bool checkSlippage(
    mcdd::BaseData& base_data);
  void angularVelocityToDirection(
    const double &ang_vel, const mcdd::Double2 &position, mcdd::Double2 &direction);
  void matrixRemoveRow(
    Eigen::MatrixXd& matrix, unsigned int rowToRemove);
  double metersToRads(
    const double& meters, const double& radius);

  Eigen::MatrixXd mat_a_, mat_vehicle_, mat_wheels_;
};
} // namespace mm_controllers