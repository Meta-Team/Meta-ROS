#include <cmath>
#include <cstdint>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <tuple>

#include "CanMessage.hpp"
#include "angles/angles.h"
#include "meta_hardware/motor_driver/dm_motor_driver.hpp"



namespace meta_hardware {

using sockcanpp::CanMessage;
using std::tuple;

DmMotor::DmMotor(const std::string &motor_model, uint32_t dm_motor_id, std::string mode,
                 double max_vel, double max_pos, double max_effort, uint32_t Kp, uint32_t Kd,
                 uint32_t Tff)
    : motor_model_(motor_model), dm_motor_id_(dm_motor_id), 
    max_vel_(max_vel), max_pos_(max_pos), max_effort_(max_effort), kp_(Kp), kd_(Kd), tff_(Tff) {

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

CanMessage DmMotor::get_motor_enable_frame(uint8_t master_id) const{
    canid_t enable_can_id = 3 << 24;
    enable_can_id |= master_id << 8;
    enable_can_id |= dm_motor_id_;
    can_frame enable_frame{.can_id = enable_can_id,
                           .can_dlc = 8,
                           .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
    return CanMessage(enable_frame);
}

CanMessage DmMotor::get_motor_disable_frame(uint8_t master_id) const{
    canid_t disable_can_id = 4 << 24;
    disable_can_id |= master_id << 8;
    disable_can_id |= dm_motor_id_;
    can_frame disable_frame{.can_id = disable_can_id,
                            .can_dlc = 8,
                            .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}};
    return CanMessage(disable_frame);
}

CanMessage DmMotor::get_motor_command_frame(double position,double velocity,double effort) const{
    uint32_t can_id_offset = 0x00;  

    if (!std::isnan(position) && std::isnan(velocity) &&
        std::isnan(effort)) {       // Position only mode
        velocity = 0.0;               // Set target velocity to 0
        effort = 0.0;                 // Set feedforward torque to 0
    } else if (std::isnan(position) && !std::isnan(velocity) &&
               std::isnan(effort)) {    // Velocity only mode
        effort = 0.0;                     // Set feedforward torque to 0
        can_id_offset = 0x100;
    } else{
        can_id_offset = 0x200;
    }

    // Only Implement the MIT mode
    
    can_frame command_frame;
    command_frame.can_id = id_ + can_id_offset; 

    auto position_raw = static_cast<uint16_t>();
    auto velocity_raw = static_cast<uint16_t>();
    auto effort_raw = static_cast<uint16_t>();

    command_frame.can_dlc = 8;

    command_frame.data[0] = position_raw >> 8;
    command_frame.data[1] = position_raw & 0xFF;

    command_frame.data[2] = velocity_raw >> 8;
    command_frame.data[3] = velocity_raw & 0xFF;

    command_frame.data[4] = Kp_raw_ >> 8;
    command_frame.data[5] = Kp_raw_ & 0xFF;

    command_frame.data[6] = Kd_raw_ >> 8;
    command_frame.data[7] = Kd_raw_ & 0xFF;

    return CanMessage(command_frame);
}


void DmMotor::set_motor_feedback(const sockcanpp::CanMessage &can_msg){

    auto error_code = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.getRawFrame().data[0]) >> 4);
    
    auto id = static_cast<uint8_t>
        ((static_cast<uint8_t>(can_msg.getRawFrame().data[0]) & 0x0F));

    auto position_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.getRawFrame().data[1]) & 0xFF) << 8) | 
        (static_cast<uint32_t>(can_msg.getRawFrame().data[2]) & 0xFF) 
        );

    auto velocity_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.getRawFrame().data[3]) & 0xFF) << 8) | 
        ((static_cast<uint32_t>(can_msg.getRawFrame().data[4]) & 0xC0) >> 12) 
        );

    auto effort_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.getRawFrame().data[4]) & 0x3F) << 4) | 
        (static_cast<uint32_t>(can_msg.getRawFrame().data[5]) & 0xFF) 
        );

    auto temperature_mos = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.getRawFrame().data[6]));

    
    auto temperature_rotor = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.getRawFrame().data[7]));


    double position = static_cast<double>(position_raw) / ((1 << 16) - 1) * 2 * max_pos_ - max_pos_;
    position_ += angles::shortest_angular_distance(position_, position);

    velocity_ = static_cast<double>(velocity_raw) / ((1 << 12) - 1) * 2 * max_vel_ - max_vel_;

    toruqe_ = static_cast<double>(effort_raw) / ((1 << 12) - 1) * 2 * max_effort_ - max_effort_;

    error_code_ = error_code;
    id_ = id;
    temperature_mos_ = temperature_mos;
    temperature_rotor_ = temperature_rotor; 

}

std::tuple<double, double, double> DmMotor::get_motor_feedback() const {
    return {position_, velocity_, toruqe_};
}

} // namespace meta_hardware
