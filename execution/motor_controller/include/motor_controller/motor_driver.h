#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "can_driver.hpp"
#include <string>
#include <tuple>

using std::string;
using std::tuple;

#define CALI_TIMEOUT 3 // seconds
#define NaN std::nan("")

class MotorDriver
{
public:
    /**
     * @brief Construct a new MotorDriver object.
     * @param rid The ROS ID of the motor.
     * @param hid The hardware ID of the motor.
     */
    MotorDriver(const string& rid, int hid) : hid(hid), rid(rid) {}

    virtual ~MotorDriver() = default;

    /**
     * @brief Get the ROS ID of the motor.
     * @return The ROS ID of the motor.
     */
    [[nodiscard]] string get_rid() { return rid; }

    /**
     * @brief Set the desired position and velocity for the motor.
     * @param goal_pos The desired position.
     * @param goal_vel The desired velocity.
     */
    virtual void set_goal(double goal_pos, double goal_vel, double goal_tor) = 0;

    /**
     * @brief Get the current state of the motor.
     * @return A tuple containing the position, velocity, and current of the motor.
     */
    [[nodiscard]] virtual tuple<double, double, double> get_state() = 0;

    /**
     * @brief Set the PID parameters for the motor.
     * @param p2v_kp The proportional gain for position to velocity.
     * @param p2v_ki The integral gain for position to velocity.
     * @param p2v_kd The derivative gain for position to velocity.
     * @param v2t_kp The proportional gain for velocity to torque.
     * @param v2t_ki The integral gain for velocity to torque.
     * @param v2t_kd The derivative gain for velocity to torque.
     */
    virtual void set_param(double p2v_kp, double p2v_ki, double p2v_kd,
        double v2t_kp, double v2t_ki, double v2t_kd) = 0;

    /**
     * @brief Print the information of the motor.
     */
    virtual void print_info() = 0;

protected:
    int hid; // hardware ID
    string rid; // ROS ID, used as a name for the motor
};

#endif // MOTOR_DRIVER_HPP