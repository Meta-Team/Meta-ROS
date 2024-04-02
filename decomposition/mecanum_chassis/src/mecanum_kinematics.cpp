#include "mecanum_chassis/mecanum_kinematics.hpp"
#include <cmath>

float MecanumKinematics::cha_wid = 0.5;
float MecanumKinematics::cha_len = 0.5;
float MecanumKinematics::wheel_angle = M_PI_4;
float MecanumKinematics::wheel_r = 0.076;
float MecanumKinematics::decel_ratio = 20.0;
float MecanumKinematics::yaw_offset = 0.0;
PidParam MecanumKinematics::cha_param;
PidOutput MecanumKinematics::cha_output;
float MecanumKinematics::yaw_error = 0.0;

motor_interface::msg::MotorGoal MecanumKinematics::chassis_decompo(const float v_x, const float v_y, const float omega)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float tangent = tan(wheel_angle);
    float L = cha_len / 2;
    float l = cha_wid / 2;
    
    add_goal(motor_goal, "LF", v_x - v_y * tangent - (L * tangent + l) * omega);
    add_goal(motor_goal, "LB", v_x + v_y * tangent - (L * tangent + l) * omega);
    add_goal(motor_goal, "RB", v_x - v_y * tangent + (L * tangent + l) * omega);
    add_goal(motor_goal, "RF", v_x + v_y * tangent + (L * tangent + l) * omega);

    return motor_goal;
}

motor_interface::msg::MotorGoal MecanumKinematics::natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, float yaw)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    
    float last_yaw_error = yaw_error;
    yaw_error = yaw - yaw_offset;

    cha_output.p = yaw_error * cha_param.kp;
    cha_output.i += yaw_error * cha_param.ki * DT;
    cha_output.d = (yaw_error - last_yaw_error) * cha_param.kd / DT;

    float omega = cha_output.sum();

    return chassis_decompo(msg->vel_x, msg->vel_y, omega);
}

void MecanumKinematics::clear_goal(motor_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void MecanumKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goal, const std::string& rid, const float goal_vel)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(goal_vel / wheel_r * decel_ratio); // convert to rad/s
    motor_goal.goal_pos.push_back(NaN);
}