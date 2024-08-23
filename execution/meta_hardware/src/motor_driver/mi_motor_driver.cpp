#include <bit>
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

MiMotor::MiMotor(const std::unordered_map<std::string, std::string> &motor_param,
                 uint8_t host_id)
    : host_id_(host_id) {
    motor_model_ = motor_param.at("motor_model");
    mi_motor_id_ = static_cast<uint8_t>(std::stoi(motor_param.at("motor_id")));

    if (motor_model_ != "CyberGear") {
        throw std::runtime_error("Unknown motor model: " + motor_model_);
    }

    std::string control_mode = motor_param.at("control_mode");
    if (control_mode == "dynamic") {
        double Kp = std::stod(motor_param.at("Kp"));
        double Kd = std::stod(motor_param.at("Kd"));
        Kp_raw_ = static_cast<uint16_t>((Kp / MAX_KP) * MAX_RAW_KP);
        Kd_raw_ = static_cast<uint16_t>((Kd / MAX_KD) * MAX_RAW_KD);
        run_mode_ = RunMode::DYNAMIC;
    } else if (control_mode == "position") {
        limit_spd_ = std::stof(motor_param.at("limit_spd"));
        loc_kp_ = std::stof(motor_param.at("loc_kp"));
        spd_kp_ = std::stof(motor_param.at("spd_kp"));
        spd_ki_ = std::stof(motor_param.at("spd_ki"));
        run_mode_ = RunMode::POSITION;
    } else if (control_mode == "velocity") {
        limit_cur_ = std::stof(motor_param.at("limit_cur"));
        spd_kp_ = std::stof(motor_param.at("spd_kp"));
        spd_ki_ = std::stof(motor_param.at("spd_ki"));
        run_mode_ = RunMode::VELOCITY;
    } else if (control_mode == "current") {
        run_mode_ = RunMode::CURRENT;
    } else {
        throw std::runtime_error("Unknown control mode: " + control_mode);
    }
}

can_frame MiMotor::motor_runmode_frame() const {
    return motor_wr_param_frame(0x7005, static_cast<uint8_t>(run_mode_));
}

can_frame MiMotor::motor_enable_frame() const {
    return std::bit_cast<can_frame>(
        mi_can_frame{.can_id = {.id = mi_motor_id_, .data = host_id_, .mode = 3},
                     .len = 8,
                     .data = {0, 0, 0, 0, 0, 0, 0, 0}});
}

can_frame MiMotor::motor_disable_frame() const {
    return std::bit_cast<can_frame>(
        mi_can_frame{.can_id = {.id = mi_motor_id_, .data = host_id_, .mode = 4},
                     .len = 8,
                     .data = {0, 0, 0, 0, 0, 0, 0, 0}});
}

can_frame MiMotor::motor_limit_frame() const {
    using enum RunMode;
    switch (run_mode_) {
    case POSITION:
        return motor_wr_param_frame(0x7017, limit_spd_);
    case VELOCITY:
        return motor_wr_param_frame(0x7018, limit_cur_);
    default:
        throw std::runtime_error("Cannot set limits for this run mode");
    }
}

can_frame MiMotor::motor_loc_kp_frame() const {
    return motor_wr_param_frame(0x701E, loc_kp_);
}

can_frame MiMotor::motor_spd_kp_frame() const {
    return motor_wr_param_frame(0x701F, spd_kp_);
}

can_frame MiMotor::motor_spd_ki_frame() const {
    return motor_wr_param_frame(0x7020, spd_ki_);
}

can_frame MiMotor::motor_dyn_frame(double position, double velocity,
                                   double effort) const {
    position = std::clamp(position, -MAX_ABS_POSITION, MAX_ABS_POSITION);
    velocity = std::clamp(velocity, -MAX_ABS_VELOCITY, MAX_ABS_VELOCITY);
    effort = std::clamp(effort, -MAX_ABS_TORQUE, MAX_ABS_TORQUE);
    auto position_raw = static_cast<uint16_t>((position / (2 * MAX_ABS_POSITION) + 0.5) *
                                              MAX_RAW_POSITION);
    auto velocity_raw = static_cast<uint16_t>((velocity / (2 * MAX_ABS_VELOCITY) + 0.5) *
                                              MAX_RAW_VELOCITY);
    auto effort_raw =
        static_cast<uint16_t>((effort / (2 * MAX_ABS_TORQUE) + 0.5) * MAX_RAW_TORQUE);

    // clang-format off
    return std::bit_cast<can_frame>(mi_can_frame{
        .can_id = {.id = mi_motor_id_, .data = effort_raw, .mode = 1},
        .len = 8,
        .data = {static_cast<uint8_t>(position_raw >> 8),
                 static_cast<uint8_t>(position_raw & 0xFF),
                 static_cast<uint8_t>(velocity_raw >> 8),
                 static_cast<uint8_t>(velocity_raw & 0xFF),
                 static_cast<uint8_t>(Kp_raw_ >> 8),
                 static_cast<uint8_t>(Kp_raw_ & 0xFF),
                 static_cast<uint8_t>(Kd_raw_ >> 8),
                 static_cast<uint8_t>(Kd_raw_ & 0xFF)}});
    // clang-format on
}

can_frame MiMotor::motor_pos_frame(double position) const {
    position = std::clamp(position, -MAX_ABS_POSITION, MAX_ABS_POSITION);

    return motor_wr_param_frame(0x7016, static_cast<float>(position));
}

can_frame MiMotor::motor_vel_frame(double velocity) const {
    velocity = std::clamp(velocity, -MAX_ABS_VELOCITY, MAX_ABS_VELOCITY);

    return motor_wr_param_frame(0x700A, static_cast<float>(velocity));
}

can_frame MiMotor::motor_wr_param_frame(uint16_t index, float value) const {
    uint32_t value_raw = std::bit_cast<uint32_t>(value);

    // clang-format off
    return std::bit_cast<can_frame>(mi_can_frame{
        .can_id = {.id = mi_motor_id_, .data = host_id_, .mode = 18},
        .len = 8,
        .data = {static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>(index >> 8),
                 0, 0,
                 static_cast<uint8_t>(value_raw & 0xFF),
                 static_cast<uint8_t>(value_raw >> 8),
                 static_cast<uint8_t>(value_raw >> 16),
                 static_cast<uint8_t>(value_raw >> 24)}});
    // clang-format on
}

can_frame MiMotor::motor_wr_param_frame(uint16_t index, uint8_t value) const {
    // clang-format off
    return std::bit_cast<can_frame>(mi_can_frame{
        .can_id = {.id = mi_motor_id_, .data = host_id_, .mode = 18},
        .len = 8,
        .data = {static_cast<uint8_t>(index & 0xFF), static_cast<uint8_t>(index >> 8),
                 0, 0,
                 value, 0, 0, 0}});
    // clang-format on
}

void MiMotor::set_motor_feedback(const mi_can_frame &can_msg) {
    auto position_raw = static_cast<uint16_t>(can_msg.data[0] << 8 | can_msg.data[1]);
    auto velocity_raw = static_cast<uint16_t>(can_msg.data[2] << 8 | can_msg.data[3]);
    auto torque_raw = static_cast<uint16_t>(can_msg.data[4] << 8 | can_msg.data[5]);

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
