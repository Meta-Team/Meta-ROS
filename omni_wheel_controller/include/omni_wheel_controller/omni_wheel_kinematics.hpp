#ifndef OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP
#define OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP

#include "Eigen/LU"

namespace omni_wheel_controller
{

class OmniWheelKinematics {
public:
  OmniWheelKinematics(std::vector<double> omni_wheel_angles,
    double omni_wheel_distance, double omni_wheel_radius);
  ~OmniWheelKinematics() = default;

  std::vector<double> forward(const std::vector<double> & wheels_vel);
  Eigen::VectorXd inverse(const double linear_x, const double linear_y, const double angular_z);
private:
  void init();
  std::vector<double> omni_wheel_angles_;
  double omni_wheel_distance_;
  double omni_wheel_radius_;
  Eigen::MatrixXd motion_mat_;
};

}  // namespace omni_wheel_controller

#endif // OMNI_WHEEL_KINEMATICS_HPP