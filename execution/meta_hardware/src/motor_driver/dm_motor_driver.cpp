#include <cmath>
#include <cstdint>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <tuple>

#include "angles/angles.h"
#include "meta_hardware/motor_driver/dm_motor_driver.hpp"

namespace meta_hardware {
DmMotor::DmMotor(const std::string &motor_model, uint32_t dm_motor_id, std::string mode,
                 double max_vel, double max_pos, double max_effort)
    : motor_model_(motor_model), dm_motor_id_(dm_motor_id), 
    max_vel_(max_vel), max_pos_(max_pos), max_effort_(max_effort) {

    uint32_t id_offeset = 0;
    if (mode == "MIT") {
        mode_ = DmMode::MIT;
    } else if (mode == "POS") {
        mode_ = DmMode::POS;
        id_offeset = 0x100;
    } else if (mode == "VEL") {
        mode_ = DmMode::VEL;
        id_offeset = 0x200;
    } else {
        throw std::runtime_error("Unknown motor mode: " + mode);
    }

    if (motor_model_ == "4310") {
        tx_can_id_ = dm_motor_id_ + id_offeset;
    } else {
        throw std::runtime_error("Unknown motor model: " + motor_model_);
    }
}

uint32_t DmMotor::get_dm_motor_id() const { return dm_motor_id_; }
canid_t DmMotor::get_tx_can_id() const { return tx_can_id_; }

void DmMotor::set_motor_feedback(uint8_t error_code, uint8_t id, 
                            uint16_t position_raw, uint16_t velocity_raw, 
                            uint16_t effort_raw, uint8_t temperature_mos, 
                            uint8_t temperature_rotor) {

    double position = static_cast<double>(position_raw - (1 << 15)) / (1 << 16) * max_pos_;
    position_ += angles::shortest_angular_distance(position_, position);
    velocity_ = static_cast<double>(velocity_raw - (1 << 11)) / (1 << 12) * max_vel_;
    toruqe_ = static_cast<double>(effort_raw - (1 << 11)) / (1 << 12) * max_effort_;
    error_code_ = error_code;
    id_ = id;
    temperature_mos_ = temperature_mos;
    temperature_rotor_ = temperature_rotor; 
}

std::tuple<double, double, double> DmMotor::get_motor_feedback() const {
    return {position_, velocity_, toruqe_};
}

} // namespace meta_hardware
