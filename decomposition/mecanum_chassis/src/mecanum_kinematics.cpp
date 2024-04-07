#include "mecanum_chassis/mecanum_kinematics.hpp"
#include <cmath>

double MecanumKinematics::cha_wid = 0.5;
double MecanumKinematics::cha_len = 0.5;
double MecanumKinematics::wheel_angle = M_PI_4;
double MecanumKinematics::wheel_r = 0.076;
double MecanumKinematics::decel_ratio = 20.0;
double MecanumKinematics::yaw_offset = 0.0;
PidParam MecanumKinematics::cha_param;
PidOutput MecanumKinematics::cha_output;
double MecanumKinematics::yaw_error = 0.0;

motor_interface::msg::MotorGoal MecanumKinematics::chassis_decompo(const double v_x, const double v_y, const double omega)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    double tangent = tan(wheel_angle);
    double L = cha_len / 2;
    double l = cha_wid / 2;
    
    add_goal(motor_goal, "LF", v_x - v_y * tangent - (L * tangent + l) * omega);
    add_goal(motor_goal, "LB", v_x + v_y * tangent - (L * tangent + l) * omega);
    add_goal(motor_goal, "RB", v_x - v_y * tangent + (L * tangent + l) * omega);
    add_goal(motor_goal, "RF", v_x + v_y * tangent + (L * tangent + l) * omega);

    return motor_goal;
}

motor_interface::msg::MotorGoal MecanumKinematics::natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double yaw)
{
    if (msg->omega == 0.0) // chassis follow the gimbal
    {
        double last_yaw_error = yaw_error;
        yaw_error = yaw - yaw_offset;

        cha_output.p = yaw_error * cha_param.kp;
        cha_output.i += yaw_error * cha_param.ki * DT;
        cha_output.d = (yaw_error - last_yaw_error) * cha_param.kd / DT;

        double follow_omega = cha_output.sum(); // omega for chassis to follow the gimbal

        return chassis_decompo(msg->vel_x, msg->vel_y, follow_omega);
    }
    else // chassis rotate independently
    {
        double dir = yaw - yaw_offset;
        double vx = msg->vel_x * cos(dir) - msg->vel_y * sin(dir);
        double vy = msg->vel_x * sin(dir) + msg->vel_y * cos(dir);

        return chassis_decompo(vx, vy, msg->omega);
    }
}

void MecanumKinematics::clear_goal(motor_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_tor.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void MecanumKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goal, const std::string& rid, const double goal_vel)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_tor.push_back(NaN);
    motor_goal.goal_vel.push_back(goal_vel / wheel_r * decel_ratio); // convert to rad/s
    motor_goal.goal_pos.push_back(NaN);
}