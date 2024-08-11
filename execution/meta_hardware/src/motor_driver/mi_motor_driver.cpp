#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <tuple>

#include "angles/angles.h"
#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/mi_motor_driver.hpp"

namespace meta_hardware {
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

using std::tuple;

MiMotor::MiMotor(const std::unordered_map<std::string, std::string> &motor_param) {
    motor_model_ = motor_param.at("motor_model");
    mi_motor_id_ = std::stoi(motor_param.at("motor_id"));

    std::string control_mode = motor_param.at("control_mode");
    double Kp = std::numeric_limits<double>::quiet_NaN();
    double Kd = std::numeric_limits<double>::quiet_NaN();
    if (control_mode == "dynamic") {
        Kp = std::stod(motor_param.at("Kp"));
        Kd = std::stod(motor_param.at("Kd"));
    }

    if (motor_model_ == "CyberGear") {
        Kp_raw_ = static_cast<uint16_t>((Kp / MAX_KP) * MAX_RAW_KP);
        Kd_raw_ = static_cast<uint16_t>((Kd / MAX_KD) * MAX_RAW_KD);
    } else {
        throw std::runtime_error("Unknown motor model: " + motor_model_);
    }
}

can_frame MiMotor::get_motor_enable_frame(uint8_t host_id) const {
    canid_t enable_can_id = 3 << 24;
    enable_can_id |= CAN_EFF_FLAG;
    enable_can_id |= host_id << 8;
    enable_can_id |= mi_motor_id_;
    return can_frame{
        .can_id = enable_can_id, .can_dlc = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
}

can_frame MiMotor::get_motor_disable_frame(uint8_t host_id) const {
    canid_t disable_can_id = 4 << 24;
    disable_can_id |= CAN_EFF_FLAG;
    disable_can_id |= host_id << 8;
    disable_can_id |= mi_motor_id_;
    return can_frame{
        .can_id = disable_can_id, .can_dlc = 8, .data = {0, 0, 0, 0, 0, 0, 0, 0}};
}

can_frame MiMotor::get_motor_dyn_frame(double position, double velocity,
                                       double effort) const {
    auto position_raw = static_cast<uint16_t>((position / (2 * MAX_ABS_POSITION) + 0.5) *
                                              MAX_RAW_POSITION);
    auto velocity_raw = static_cast<uint16_t>((velocity / (2 * MAX_ABS_VELOCITY) + 0.5) *
                                              MAX_RAW_VELOCITY);
    auto effort_raw =
        static_cast<uint16_t>((effort / (2 * MAX_ABS_TORQUE) + 0.5) * MAX_RAW_TORQUE);

    can_frame command_frame;
    command_frame.can_id = 1 << 24;
    command_frame.can_id |= CAN_EFF_FLAG;
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

    return command_frame;
}

can_frame MiMotor::get_motor_pos_frame(double position) const {
    throw std::runtime_error("Position control mode not implemented");
}

can_frame MiMotor::get_motor_vel_frame(double velocity) const {
    throw std::runtime_error("Velocity control mode not implemented");
}

void MiMotor::set_motor_feedback(const can_frame &can_msg) {
    auto position_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[0]) << 8) |
                              static_cast<uint16_t>(can_msg.data[1]));
    auto velocity_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[2]) << 8) |
                              static_cast<uint16_t>(can_msg.data[3]));
    auto torque_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[4]) << 8) |
                              static_cast<uint16_t>(can_msg.data[5]));

    position_ = (position_raw / double(MAX_RAW_POSITION)) * (2 * MAX_ABS_POSITION) -
                MAX_ABS_POSITION;

    velocity_ = (velocity_raw / double(MAX_RAW_VELOCITY)) * (2 * MAX_ABS_VELOCITY) -
                MAX_ABS_VELOCITY;

    torque_ =
        (torque_raw / double(MAX_RAW_TORQUE)) * (2 * MAX_ABS_TORQUE) - MAX_ABS_TORQUE;
}

tuple<double, double, double> MiMotor::get_motor_feedback() const {
    return {position_, velocity_, torque_};
}

} // namespace meta_hardware
