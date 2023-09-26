#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "motor_interface/msg/dji_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/msg/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter

enum WheelId
{
    F = 0,
    L = 1,
    B = 2,
    R = 3
};

namespace OmniKinematics
{
    motor_interface::msg::DjiGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    void set_goal(motor_interface::msg::DjiGoal &motor_goals,
                  const WheelId wheel_id, const float goal_vel, const float goal_pos);
};

#endif // OMNI_KINEMATICS_HPP