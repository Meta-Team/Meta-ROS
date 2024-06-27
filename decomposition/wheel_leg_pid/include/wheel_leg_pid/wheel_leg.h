#ifndef WHEEL_LEG_PID_H
#define WHEEL_LEG_PID_H

#include "pid_algorithm.hpp"
#include <memory>

#include "device_interface/msg/motor_goal.hpp"

using device_interface::msg::MotorGoal;
using Param = PidAlgorithm::PidParam;
using std::unique_ptr;

class WheelLeg
{
public:
    /**
     * @brief Construct a new WheelLeg object.
     * @param omega_wv_p The PID parameters for body angular velocity to wheel velocity.
     * @param obli_wv_p The PID parameters for oblique angle to wheel velocity.
     * @param bv_obli_p The PID parameters for body velocity to oblique angle.
     */
    WheelLeg(Param omega_wv_p, Param obli_wv_p, Param bv_obli_p);

    /**
     * @brief Destroy the WheelLeg object.
     * @note Stop the PID algorithms.
     */
    ~WheelLeg();

    /**
     * @brief Input the body angular velocity feedback.
     * @param omega The body angular velocity in rad/s.
     */
    void omega_feedback(double omega);

    /**
     * @brief Input the body velocity feedback.
     * @param bv The body velocity in m/s.
     */
    void bv_feedback(double bv);

    /**
     * @brief Input the oblique angle feedback.
     * @param obli The oblique angle in rad.
     */
    void obli_feedback(double obli);

    /**
     * @brief Set the body velocity goal.
     * @param bv The body velocity goal in m/s.
     * @param omega The body angular velocity goal in rad/s.
     */
    void set_goal(double bv, double omega);

    /**
     * @brief Get the motor goal message.
     * @return The motor goal message.
     */
    [[nodiscard]] MotorGoal get_msg() const;

private:
    unique_ptr<PidAlgorithm> omega_wv; // body angular velocity to wheel velocity
    unique_ptr<PidAlgorithm> obli_wv; // oblique angle to wheel velocity
    unique_ptr<PidAlgorithm> bv_obli; // body velocity to oblique angle, the angle is proportional to acceleration

    bool omega_ready = false;
    bool obli_ready = false;
    bool bv_ready = false;

    /**
     * @brief A loop to pass oblique angle from bv_obli to obli_wv.
     * @note The frequency is 100 Hz.
     */
    void pass_vel_loop();

    /**
     * @brief Add a wheel goal to the motor goal message.
     */
    static void add_wheel_goal(MotorGoal& msg, const std::string& motor_id, double goal_vel);

    /**
     * @brief Set the wheel velocity goal.
     */
    void set_wheel_vel(MotorGoal& msg) const;

    /**
     * @brief Add a joint goal to the motor goal message.
     */
    static void add_joint_goal(MotorGoal& msg, const std::string& motor_id, double goal_pos);

    /**
     * @brief Set the leg length goal.
     */
    void set_leg_len(MotorGoal& msg) const;

    /**
     * @brief Convert the leg length to the motor angle.
     * @param len The leg length in m.
     * @return The joint angle in rad.
     */
    [[nodiscard]] double len_to_angle(double len) const;
};

#endif // WHEEL_LEG_PID_H