#include "meta_chassis_controller/omni_wheel_kinematics.hpp"
#include <iostream>

namespace meta_chassis_controller {
OmniWheelKinematics::OmniWheelKinematics(const std::vector<double> &omni_wheel_forward_angles,
                        const std::vector<double> &omni_wheel_center_x,
                        const std::vector<double> &omni_wheel_center_y,
                        const std::vector<double> &omni_wheel_sliding_angles,
                        double omni_wheel_radius)
    : omni_wheel_forward_angles_(omni_wheel_forward_angles),
    omni_wheel_center_x_(omni_wheel_center_x),
    omni_wheel_center_y_(omni_wheel_center_y),
    omni_wheel_sliding_angles_(omni_wheel_sliding_angles),
    omni_wheel_radius_(omni_wheel_radius) {
    init();
}

void OmniWheelKinematics::init() {
    auto degree2rad = [](double degree) -> double {
        return degree * M_PI / 180.0;
    };

    motion_mat_ = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(omni_wheel_forward_angles_.size()), 3);
    Eigen::MatrixXd decompose_mat(2, 3);
    Eigen::MatrixXd rotation_mat(2, 2);
    Eigen::MatrixXd translation_mat(1, 2);
    for (size_t i = 0; i < omni_wheel_forward_angles_.size(); ++i) {
        decompose_mat(0, 0) = 1;
        decompose_mat(0, 1) = 0;
        decompose_mat(0, 2) = -omni_wheel_center_y_[i];
        decompose_mat(1, 0) = 0;
        decompose_mat(1, 1) = 1;
        decompose_mat(1, 2) = omni_wheel_center_x_[i];
        rotation_mat(0, 0) = cos(degree2rad(omni_wheel_forward_angles_[i]));
        rotation_mat(0, 1) = sin(degree2rad(omni_wheel_forward_angles_[i]));
        rotation_mat(1, 0) = -sin(degree2rad(omni_wheel_forward_angles_[i]));
        rotation_mat(1, 1) = cos(degree2rad(omni_wheel_forward_angles_[i]));
        translation_mat(0, 0) = 1;
        translation_mat(0, 1) = tan(degree2rad(omni_wheel_sliding_angles_[i]));
        motion_mat_.row(static_cast<Eigen::Index>(i)) = translation_mat * rotation_mat * decompose_mat;
    }
    motion_mat_ /= omni_wheel_radius_;
    std::cout << "motion_mat_ = " << std::endl << motion_mat_ << std::endl;
}

Eigen::VectorXd OmniWheelKinematics::inverse(Eigen::Vector3d twist) const {
    std::vector<double> wheels_vel(omni_wheel_forward_angles_.size(), 0.0);
    Eigen::VectorXd wheels_vel_eigen = motion_mat_ * twist;
    return wheels_vel_eigen;
}

} // namespace meta_chassis_controller