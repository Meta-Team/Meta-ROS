#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/msg/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter
#define LF_D 0
#define RF_D 1
#define LB_D 2
#define RB_D 3
#define LF_V 4
#define RF_V 5
#define LB_V 6
#define RB_V 7

namespace AgvKinematics
{
    motor_interface::msg::MotorGoal natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg,
                                                    float yaw_diff);
    motor_interface::msg::MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    void add_goal(motor_interface::msg::MotorGoal &motor_goals,
                  const int motor_id, const float goal_vel, const float goal_pos);

    void clear_goals(motor_interface::msg::MotorGoal &motor_goals);

    float rss(float x, float y); // root sum square
};

#endif // AGV_KINEMATICS_HPP