#ifndef METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP
#define METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP

#include "Eigen/LU"

namespace meta_chassis_controller {

class AgvWheelKinematics {
  public:
    AgvWheelKinematics(const double agv_wheel_center_x, const double agv_wheel_center_y, const double agv_wheel_radius); 
    ~AgvWheelKinematics() = default;

    // std::vector<double> forward(const std::vector<double> &wheels_vel);
    void inverse(Eigen::VectorXd, std::vector<double> & curr_pos, std::vector<double> & curr_vel);
    
  private:
    void init();
    double agv_wheel_center_x_, agv_wheel_center_y_, agv_wheel_radius_, agv_radius_;
    std::tuple<double, double> get_output(double curr_pos, double curr_vel, double target_pos, double target_vel);
    // Eigen::MatrixXd motion_mat_;
};

} // namespace meta_chassis_controller

#endif // METAV_CHASSIS_CONTROLLER__OMNI_WHEEL_KINEMATICS_HPP