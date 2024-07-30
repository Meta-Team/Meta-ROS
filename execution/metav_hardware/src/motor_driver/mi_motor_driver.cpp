#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <tuple>

#include "CanMessage.hpp"
#include "angles/angles.h"
#include "metav_hardware/motor_driver/mi_motor_driver.hpp"

namespace metav_hardware {
constexpr uint16_t MAX_RAW_TORQUE = 65535;
constexpr uint16_t MAX_RAW_VELOCITY = 65535;
constexpr uint16_t MAX_RAW_POSITION = 65535;
constexpr uint16_t MAX_RAW_KP = 65535;
constexpr uint16_t MAX_RAW_KD = 65535;
constexpr double MAX_ABS_TORQUE = 12.0;
constexpr double MAX_ABS_VELOCITY = 30.0;
constexpr double MAX_ABS_POSITION = 4.0 * M_PI;
constexpr double MAX_KP = 500.0;
constexpr double MAX_KD = 5.0;

MiMotor::MiMotor(const std::string &motor_model, uint8_t mi_motor_id, double Kp,
                 double Kd)
    : motor_model_(motor_model), mi_motor_id_(mi_motor_id) {
    if (motor_model_ == "CyberGear") {
        Kp_raw_ = static_cast<uint16_t>((Kp / MAX_KP) * MAX_RAW_KP);
        Kd_raw_ = static_cast<uint16_t>((Kd / MAX_KD) * MAX_RAW_KD);
    } else {
        throw std::runtime_error("Unknown motor model: " + motor_model_);
    }
}

sockcanpp::CanMessage MiMotor::get_motor_enable_frame(uint8_t host_id) const {
    canid_t enable_can_id = 3 << 24;
    enable_can_id |= host_id << 8;
    enable_can_id |= mi_motor_id_;
    can_frame enable_frame{.can_id = enable_can_id,
                           .can_dlc = 8,
                           .data = {0, 0, 0, 0, 0, 0, 0, 0}};
    return sockcanpp::CanMessage(enable_frame);
}

sockcanpp::CanMessage MiMotor::get_motor_disable_frame(uint8_t host_id) const {
    canid_t disable_can_id = 4 << 24;
    disable_can_id |= host_id << 8;
    disable_can_id |= mi_motor_id_;
    can_frame disable_frame{.can_id = disable_can_id,
                            .can_dlc = 8,
                            .data = {0, 0, 0, 0, 0, 0, 0, 0}};
    return sockcanpp::CanMessage(disable_frame);
}

sockcanpp::CanMessage MiMotor::get_motor_command_frame(double position,
                                                       double velocity,
                                                       double effort) const {
    if (!std::isnan(position) && std::isnan(velocity) &&
        std::isnan(effort)) { // Position only mode
        velocity = 0.0;       // Set target velocity to 0
        effort = 0.0;         // Set feedforward torque to 0
    } else if (std::isnan(position) && !std::isnan(velocity) &&
               std::isnan(effort)) { // Velocity only mode
        effort = 0.0;                // Set feedforward torque to 0
    }

    auto position_raw = static_cast<uint16_t>(
        (position / (2 * MAX_ABS_POSITION) + 0.5) * MAX_RAW_POSITION);
    auto velocity_raw = static_cast<uint16_t>(
        (velocity / (2 * MAX_ABS_VELOCITY) + 0.5) * MAX_RAW_VELOCITY);
    auto effort_raw = static_cast<uint16_t>(
        (effort / (2 * MAX_ABS_TORQUE) + 0.5) * MAX_RAW_TORQUE);

    can_frame command_frame;
    command_frame.can_id = 1 << 24;
    command_frame.can_id |= effort_raw << 8;
    command_frame.can_id |= mi_motor_id_;

    command_frame.can_dlc = 8;

    command_frame.data[0] = position_raw >> 8;
    command_frame.data[1] = position_raw & 0xFF;

    command_frame.data[2] = velocity_raw >> 8;
    command_frame.data[3] = velocity_raw & 0xFF;

    command_frame.data[4] = Kp_raw_ >> 8;
    command_frame.data[5] = Kp_raw_ & 0xFF;

    command_frame.data[6] = Kd_raw_ >> 8;
    command_frame.data[7] = Kd_raw_ & 0xFF;

    return sockcanpp::CanMessage(command_frame);
}

void MiMotor::set_motor_feedback(const sockcanpp::CanMessage &can_msg) {
    auto position_raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(can_msg.getRawFrame().data[0]) << 8) |
        static_cast<uint16_t>(can_msg.getRawFrame().data[1]));
    auto velocity_raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(can_msg.getRawFrame().data[2]) << 8) |
        static_cast<uint16_t>(can_msg.getRawFrame().data[3]));
    auto torque_raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(can_msg.getRawFrame().data[4]) << 8) |
        static_cast<uint16_t>(can_msg.getRawFrame().data[5]));

    double position =
        (position_raw / double(MAX_RAW_POSITION)) * (2 * MAX_ABS_POSITION) -
        MAX_ABS_POSITION;
    position_ += angles::shortest_angular_distance(
        position_, position); // Accumulate angles

    velocity_ =
        (velocity_raw / double(MAX_RAW_VELOCITY)) * (2 * MAX_ABS_VELOCITY) -
        MAX_ABS_VELOCITY;

    torque_ = (torque_raw / double(MAX_RAW_TORQUE)) * (2 * MAX_ABS_TORQUE) -
              MAX_ABS_TORQUE;
}

std::tuple<double, double, double> MiMotor::get_motor_feedback() const {
    return {position_, velocity_, torque_};
}

} // namespace metav_hardware
