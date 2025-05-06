#ifndef METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP
#define METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP

#include "Eigen/LU"

namespace meta_chassis_controller {

class AgvWheelKinematics {
  public:
    AgvWheelKinematics(const double agv_wheel_center_x, const double agv_wheel_center_y, const double agv_wheel_radius); 
    ~AgvWheelKinematics() = default;

    std::pair<std::array<double, 4>, std::array<double, 4>> inverse(Eigen::VectorXd twist, const std::array<double, 4> & curr_pos);

  private:
    void init();
    double agv_wheel_center_x_, agv_wheel_center_y_, agv_wheel_radius_, agv_radius_;
    std::pair<double, double> xy2polar(double curr_pos, double target_x, double target_y);
};

} // namespace meta_chassis_controller

#endif // METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP