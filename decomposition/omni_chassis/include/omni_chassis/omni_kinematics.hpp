#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/msg/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter
#define F 0
#define L 1
#define B 2
#define R 3

namespace OmniKinematics
{
    motor_interface::msg::MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    void add_goal(motor_interface::msg::MotorGoal &motor_goals,
                  const int motor_id, const float goal_vel, const float goal_pos);

    void clear_goals(motor_interface::msg::MotorGoal &motor_goals);
};

#endif // OMNI_KINEMATICS_HPP