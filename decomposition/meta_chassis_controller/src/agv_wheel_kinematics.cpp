#include <csignal>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <tuple>

#include "meta_chassis_controller/agv_wheel_kinematics.hpp"
#include "angles/angles.h"


namespace meta_chassis_controller {
AgvWheelKinematics::AgvWheelKinematics(const double agv_wheel_center_x, const double agv_wheel_center_y, const double agv_wheel_radius) :
    agv_wheel_center_x_(agv_wheel_center_x), agv_wheel_center_y_(agv_wheel_center_y), agv_wheel_radius_(agv_wheel_radius) {
    agv_radius_ = sqrt(agv_wheel_center_x * agv_wheel_center_x + agv_wheel_center_y * agv_wheel_center_y);
}

std::pair<double,double> AgvWheelKinematics::xy2polar(double curr_pos, double curr_vel, double target_x, double target_y) {
    double target_vel = sqrt(target_x * target_x + target_y * target_y) / agv_wheel_radius_;

    // If the target velocity is zero, atan2 is meaningless, we should preserve current position
    if (target_vel == 0.0) {
        return {curr_pos, 0.0};
    }

    double target_angle = atan2(target_y, target_x);
    double angle_diff = angles::shortest_angular_distance(curr_pos, target_angle);

    if (angle_diff < (M_PI / 2) && angle_diff > -(M_PI / 2) ) {       // FIXME: This is not correct here
        return {target_angle, target_vel};
    } else {
        return {angles::normalize_angle(M_PI + target_angle), -target_vel};
    }
}

std::pair<std::array<double, 4>, std::array<double, 4>> AgvWheelKinematics::inverse(Eigen::VectorXd twist
    // , const std::array<double, 4> & curr_pos, const std::array<double, 4> & curr_vel
) {
    // twist: [vx, vy, wz]
    // curr_pos: [left_forward, left_back, right_forward, right_back]
    // curr_vel: [left_forward, left_back, right_forward, right_back]

    double vx = twist[0];
    double vy = twist[1];
    double wz = twist[2];
    double rot_vel = wz * agv_radius_;

    double left_foward_x = vx - rot_vel;
    double left_foward_y = vy + rot_vel;
    double left_back_x = vx - rot_vel;
    double left_back_y = vy - rot_vel;
    double right_forward_x = vx + rot_vel;
    double right_forward_y = vy + rot_vel;
    double right_back_x = vx + rot_vel;
    double right_back_y = vy - rot_vel;

    const auto [left_forward_pos, left_forward_vel] = xy2polar(curr_target_pos_[0], curr_target_vel_[0], left_foward_x, left_foward_y);
    const auto [left_back_pos, left_back_vel] = xy2polar(curr_target_pos_[1], curr_target_vel_[1], left_back_x, left_back_y);
    const auto [right_forward_pos, right_forward_vel] = xy2polar(curr_target_pos_[2], curr_target_vel_[2], right_forward_x, right_forward_y);
    const auto [right_back_pos, right_back_vel] = xy2polar(curr_target_pos_[3], curr_target_vel_[3], right_back_x, right_back_y);

    std::array<double, 4> target_pos = {left_forward_pos, left_back_pos, right_forward_pos, right_back_pos};
    std::array<double, 4> target_vel = {left_forward_vel, left_back_vel, right_forward_vel, right_back_vel};

    curr_target_pos_ = target_pos;
    curr_target_vel_ = target_vel;

    return {target_pos, target_vel};
}

} // namespace meta_chassis_controller