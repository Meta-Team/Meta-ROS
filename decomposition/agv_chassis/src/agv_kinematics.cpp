#include "agv_chassis/agv_kinematics.hpp"

motor_interface::msg::MotorGoal AgvKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr nat_msg)
{
    motor_interface::msg::MotorGoal motor_goals;
    // TODO: implement natural_decompo
    return motor_goals;
}

motor_interface::msg::MotorGoal AgvKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg)
{
    motor_interface::msg::MotorGoal motor_goals;
    // TODO: implement absolute_decompo
    return motor_goals;
}