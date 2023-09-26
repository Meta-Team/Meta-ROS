#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "motor_interface/msg/dji_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/msg/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter

enum DirectionMotor
{
    LF_D,
    RF_D,
    LB_D,
    RB_D
};

enum VelocityMotor
{
    LF_V,
    RF_V,
    LB_V,
    RB_V
};

namespace AgvKinematics
{
    motor_interface::msg::MotorGoal natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg,
                                                    float yaw_diff);
    motor_interface::msg::MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    void add_goal(motor_interface::msg::MotorGoal &motor_goals,
                  const int motor_id, const float goal_vel, const float goal_pos);

    void set_dji_goal(motor_interface::msg::DjiGoal &motor_goals, const VelocityMotor motor_id, const float goal_pos);

    void clear_goals(motor_interface::msg::MotorGoal &motor_goals);

    float rss(float x, float y); // root sum square
};

#endif // AGV_KINEMATICS_HPP