#include "omni_wheel_controller/omni_wheel_kinematics.hpp"
#include <iostream>

namespace omni_wheel_controller
{
OmniWheelKinematics::OmniWheelKinematics(std::vector<double> omni_wheel_angles, double omni_wheel_distance,
                                         double omni_wheel_radius)
  : omni_wheel_angles_(omni_wheel_angles), omni_wheel_distance_(omni_wheel_distance), omni_wheel_radius_(omni_wheel_radius)
{
  init();
}

void OmniWheelKinematics::init()
{
  motion_mat_ = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(omni_wheel_angles_.size()), 3);
  for (size_t i = 0; i < omni_wheel_angles_.size(); i++)
  {
    motion_mat_(static_cast<Eigen::Index>(i), 0) = cos((omni_wheel_angles_[i] - 90.0) / 180.0 * M_PI);
    motion_mat_(static_cast<Eigen::Index>(i), 1) = sin((omni_wheel_angles_[i] - 90.0) / 180.0 * M_PI);
    motion_mat_(static_cast<Eigen::Index>(i), 2) = - omni_wheel_distance_;
  }
  std::cout << "motion_mat_ = " << std::endl << motion_mat_ << std::endl;
  motion_mat_ /= omni_wheel_radius_;
}

Eigen::VectorXd OmniWheelKinematics::inverse(Eigen::Vector3d twist)
{
  std::vector<double> wheels_vel(omni_wheel_angles_.size(), 0.0);
  Eigen::VectorXd wheels_vel_eigen = motion_mat_ * twist;
  return wheels_vel_eigen;
}

}  // namespace omni_wheel_controller