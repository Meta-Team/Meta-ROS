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
    // init();
}

// void AgvWheelKinematics::init() {
    
// }

std::tuple<double,double> AgvWheelKinematics::get_output(double curr_pos, double curr_vel, double target_pos, double target_vel){
    if(target_vel == 0.0){
        return {curr_pos, 0.0};
    }

    double angle_diff = angles::shortest_angular_distance(curr_pos, target_pos);
    if(curr_vel > 0 && angle_diff < M_PI / 2 && angle_diff > - M_PI / 2){       // FIXME: This is not correct here
        return {target_pos, target_vel};
    }else if(curr_vel < 0 && (angle_diff > M_PI / 2 || angle_diff < - M_PI / 2)){
        return {target_pos, target_vel};
    }else{
        return {angles::normalize_angle(M_PI * 2 - target_pos), -target_vel};
    }
}

void AgvWheelKinematics::inverse(Eigen::VectorXd twist, std::vector<double> & curr_pos, std::vector<double> & curr_vel){
    // twist: [vx, vy, wz]
    // curr_pos: [left_forward, left_back, right_forward, right_back]
    // curr_vel: [left_forward, left_back, right_forward, right_back]

    if ( curr_pos.size() != 4 || curr_pos.size() != 4){
        // RCLCPP_ERROR();
        // std::raise(ERROR); // Raise an error
    }
    double vx = twist[0];
    double vy = twist[1];
    double wz = twist[2];
    double v = sqrt(vx * vx + vy * vy) / agv_wheel_radius_;
    double rot_vel = wz * agv_radius_;
    
    double left_foward_x = vx - rot_vel;
    double left_foward_y = vy + rot_vel;
    double left_back_x = vx - rot_vel;
    double left_back_y = vy - rot_vel;
    double right_forward_x = vx + rot_vel;
    double right_forward_y = vy + rot_vel;
    double right_back_x = vx + rot_vel;
    double right_back_y = vy - rot_vel;

    double left_forward_angle = atan2(left_foward_y, left_foward_x);
    double left_back_angle = atan2(left_back_y, left_back_x);
    double right_forward_angle = atan2(right_forward_y, right_forward_x);
    double right_back_angle = atan2(right_back_y, right_back_x);

    // const auto& [curr_pos[0], curr_vel[0]] = get_output(curr_pos[0], curr_vel[0], left_forward_angle, v);
    // const auto& [curr_pos[1], curr_vel[1]] = get_output(curr_pos[1], curr_vel[1], left_back_angle, v);
    // const auto& [curr_pos[2], curr_vel[2]] = get_output(curr_pos[2], curr_vel[2], right_forward_angle, v);
    // const auto& [curr_pos[3], curr_vel[3]] = get_output(curr_pos[3], curr_vel[3], right_back_angle, v);

    const auto& [left_forward_pos, left_forward_vel] = get_output(curr_pos[0], curr_vel[0], left_forward_angle, v);
    const auto& [left_back_pos, left_back_vel] = get_output(curr_pos[1], curr_vel[1], left_back_angle, v);
    const auto& [right_forward_pos, right_forward_vel] = get_output(curr_pos[2], curr_vel[2], right_forward_angle, v);
    const auto& [right_back_pos, right_back_vel] = get_output(curr_pos[3], curr_vel[3], right_back_angle, v);

    curr_pos[0] = left_forward_pos;
    curr_pos[1] = left_back_pos;
    curr_pos[2] = right_forward_pos;
    curr_pos[3] = right_back_pos;
    curr_vel[0] = left_forward_vel;
    curr_vel[1] = left_back_vel;
    curr_vel[2] = right_forward_vel;
    curr_vel[3] = right_back_vel;
    


}   

} // namespace meta_chassis_controller