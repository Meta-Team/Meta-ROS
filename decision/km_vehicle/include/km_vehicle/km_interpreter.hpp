#ifndef KM_INTERPRETER_HPP
#define KM_INTERPRETER_HPP

#include "operation_interface/msg/key_mouse.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "vision_interface/msg/auto_aim.hpp"
#include <cstdint>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#define PERIOD 5 // ms

using operation_interface::msg::KeyMouse;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;
using vision_interface::msg::AutoAim;

/**
 * @class KmInterpreter
 * @brief A class to interpret keymouse control inputs for a vehicle.
 * When auto aim is enabled, operator can still manually interfere the aim.
 * @note There is an internal thread to regularly interpret the inputs.
 */
class KmInterpreter
{
public:
    /**
     * @brief The aim mode.
     * Manual or Auto.
     */
    enum AimMode
    {
        MANUAL,
        AUTO
    };

    /**
    * @brief Constructs a new KmInterpreter object.
    * @param max_vel The maximum velocity for moving.
    * @param max_omega The maximum angular velocity for moving.
    * @param aim_sens The sensitivity for aiming.
    * @param interfere_sens The sensitivity for the interference when auto aim is active.
    */
    KmInterpreter(double max_vel, double max_omega, double aim_sens, double interfere_sens);

    /**
     * @brief Destroy the object
     */
    ~KmInterpreter();
    
    /**
     * @brief Set the manual input from the keymouse control.
     * @param msg The keymouse control message.
     */
    void manual_input(const KeyMouse::SharedPtr msg);

    /**
     * @brief Set the auto aim input from the vision system.
     * @param msg The auto aim message.
     */
    void vision_input(const AutoAim::SharedPtr msg);

    /**
     * @brief Get the move msg.
     * @return The move object.
     */
    [[nodiscard]] Move get_move() const { return *move_; }

    /**
     * @brief Get the shoot msg.
     * @return The shoot object.
     */
    [[nodiscard]] Shoot get_shoot() const { return *shoot_; }

    /**
     * @brief Get the aim msg.
     * @return The aim object.
     */
    [[nodiscard]] Aim get_aim() const { return *aim_; }

    /**
     * @brief Check if the node should publish the messages.
     * @return True if active.
     */
    [[nodiscard]] bool is_active() const { return active; }


private:
    // publish only when active
    bool active = false;

    // buttons and axes
    bool w, a, s, d, shift, ctrl, q, e, r, f, g, z, x, c, v, b;
    bool left_button, right_button;
    double mouse_x, mouse_y, mouse_z;

    // params
    double max_vel, max_omega, max_feed, mac_shoot, aim_sens, itf_sens;

    // buffers
    double manual_yaw = 0, manual_pitch = 0;
    double auto_yaw = 0, auto_pitch = 0;

    // messages
    Move::SharedPtr move_;
    Shoot::SharedPtr shoot_;
    Aim::SharedPtr aim_;

    std::thread interpret_thread;
    double last_auto_time = 0; ///< The last time auto aim is active.

    AimMode mode = MANUAL;

    /**
     * @brief Interpret the inputs and update the move, shoot, and aim messages.
     * @note This function is called by the internal thread.
     */
    void interpret();

    /**
     * @brief Limit the value within the range [-max_val, max_val].
     * @param val The value to be limited.
     * @param max_val The maximum value.
     */
    void curb(double& val, double max_val);

    /**
     * @brief Similar to fmod, but limited within the range [0, mod).
     * @param val The value to be modified.
     * @param mod The amount to be added.
     */
    void my_mod(double& val, double mod);
};

#endif // KM_INTERPRETER_HPP