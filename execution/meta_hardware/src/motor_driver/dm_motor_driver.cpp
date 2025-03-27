#include <bit>
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

using std::tuple;

constexpr double MAX_KP = 500.0;
constexpr double MIN_KP = 0.0;
constexpr double MAX_KD = 5.0;
constexpr double MIN_KD = 0.0;

DmMotor::DmMotor(const std::unordered_map<std::string, std::string> &motor_param,
                 uint8_t master_id) : master_id_(master_id) {

    motor_model_ = motor_param.at("motor_model");
    dm_motor_id_ = static_cast<uint8_t>(std::stoi(motor_param.at("motor_id")));

    max_pos_ = std::stod(motor_param.at("max_pos"));
    max_vel_ = std::stod(motor_param.at("max_vel"));
    max_effort_ = std::stod(motor_param.at("max_effort"));

    std::string control_mode = motor_param.at("control_mode");
    uint32_t id_offset = 0;

    if (control_mode == "mit") {
        Kp_ = std::stod(motor_param.at("Kp"));
        Kd_ = std::stod(motor_param.at("Kd"));

        Kp_raw_ = static_cast<uint16_t>((Kp_ - MIN_KP)/(MAX_KP - MIN_KP) * ( (1 << 12) - 1));
        Kd_raw_ = static_cast<uint16_t>((Kd_ - MIN_KD)/(MAX_KD - MIN_KD) * ( (1 << 12) - 1));

        run_mode_ = RunMode::MIT;

    } else if (control_mode == "position") {
        run_mode_ = RunMode::POSITION;
        id_offset = 0x100;
    } else if (control_mode == "velocity") {
        run_mode_ = RunMode::VELOCITY;
        id_offset = 0x200;
    } else {
        throw std::runtime_error("Unknown motor mode: " + control_mode);
    }

    if (motor_model_ == "DaMiao") {
        tx_can_id_ = dm_motor_id_ + id_offset;
    } else {
        throw std::runtime_error("Unknown motor model: " + motor_model_);
    }
}

uint32_t DmMotor::get_dm_motor_id() const { return dm_motor_id_; }
canid_t DmMotor::get_tx_can_id() const { return tx_can_id_; }

can_frame DmMotor::motor_enable_frame() const{
    canid_t enable_can_id = tx_can_id_;
    can_frame enable_frame{.can_id = enable_can_id,
                           .can_dlc = 8,
                           .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}};
    return enable_frame;
}

can_frame DmMotor::motor_disable_frame() const{
    canid_t disable_can_id = tx_can_id_;
    can_frame disable_frame{.can_id = disable_can_id,
                            .can_dlc = 8,
                            .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}};
    return disable_frame;
}

can_frame DmMotor::motor_save_initial_frame() const{
    canid_t disable_can_id = tx_can_id_;
    can_frame disable_frame{.can_id = disable_can_id,
                            .can_dlc = 8,
                            .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}};
    return disable_frame;
}

can_frame DmMotor::motor_clear_error_frame() const{
    canid_t disable_can_id = tx_can_id_;
    can_frame disable_frame{.can_id = disable_can_id,
                            .can_dlc = 8,
                            .data = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}};
    return disable_frame;
}

can_frame DmMotor::motor_mit_frame(double position, double velocity, double effort) const{
    uint16_t velocity_raw = double_to_raw(velocity, max_vel_, -max_vel_, 12);
    uint16_t position_raw = double_to_raw(position, max_pos_, -max_pos_, 16);
    uint16_t effort_raw = double_to_raw(effort, max_effort_, -max_effort_, 12);
    can_frame frame = {
        .can_id = tx_can_id_,
        .can_dlc = 8,
        .data = { static_cast<uint8_t>((position_raw & 0xFF00) >> 8),
                  static_cast<uint8_t>(position_raw & 0x00FF), 
                  static_cast<uint8_t>((velocity_raw & 0x0FF0) >> 4),
                  static_cast<uint8_t>(((velocity_raw & 0x000F) << 4) | ((Kp_raw_ & 0x0F00) >> 8)), 
                  static_cast<uint8_t>(Kp_raw_ & 0x00FF),
                  static_cast<uint8_t>((Kd_raw_ & 0x0FF0) >> 4),
                  static_cast<uint8_t>(((Kd_raw_ & 0x000F) << 4) | ((effort_raw & 0x0F00) >> 8)), 
                  static_cast<uint8_t>(effort_raw & 0x00FF)}
    };
    return frame;
}

can_frame DmMotor::motor_pos_frame(double position) const{
    double velocity = max_vel_;
    uint32_t velocity_raw = std::bit_cast<uint32_t>(static_cast<float>(velocity));
    uint32_t position_raw = std::bit_cast<uint32_t>(static_cast<float>(position));
    can_frame frame = {
        .can_id = tx_can_id_,
        .can_dlc = 8,
        .data = {   static_cast<uint8_t>((position_raw & 0x000000FF)),
                    static_cast<uint8_t>((position_raw & 0x0000FF00) >> 8),
                    static_cast<uint8_t>((position_raw & 0x00FF0000) >> 16), 
                    static_cast<uint8_t>((position_raw & 0xFF000000) >> 24),
                    static_cast<uint8_t>((velocity_raw & 0x000000FF)),
                    static_cast<uint8_t>((velocity_raw & 0x0000FF00) >> 8),
                    static_cast<uint8_t>((velocity_raw & 0x00FF0000) >> 16), 
                    static_cast<uint8_t>((velocity_raw & 0xFF000000) >> 24)}
    };
    return frame;
}

can_frame DmMotor::motor_vel_frame(double velocity) const{
    uint32_t velocity_raw = std::bit_cast<uint32_t>(static_cast<float>(velocity));
    can_frame frame = {
        .can_id = tx_can_id_,
        .can_dlc = 4,
        .data = {   static_cast<uint8_t>((velocity_raw & 0x000000FF)),
                    static_cast<uint8_t>((velocity_raw & 0x0000FF00) >> 8),
                    static_cast<uint8_t>((velocity_raw & 0x00FF0000) >> 16), 
                    static_cast<uint8_t>((velocity_raw & 0xFF000000) >> 24)}
    };
    return frame;
}

void DmMotor::set_motor_feedback(const can_frame &can_msg){

    auto error_code = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.data[0]) >> 4);
    
    auto id = static_cast<uint8_t>
        ((static_cast<uint8_t>(can_msg.data[0]) & 0x0F));

    auto position_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.data[1]) & 0xFF) << 8) | 
        (static_cast<uint32_t>(can_msg.data[2]) & 0xFF) 
        );

    auto velocity_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.data[3]) & 0xFF) << 4) | 
        ((static_cast<uint32_t>(can_msg.data[4]) & 0xF0) >> 4) 
        );

    auto effort_raw = static_cast<int32_t>
        (((static_cast<uint32_t>(can_msg.data[4]) & 0x0F) << 8) | 
        (static_cast<uint32_t>(can_msg.data[5]) & 0xFF) 
        );

    auto temperature_mos = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.data[6]));

    
    auto temperature_rotor = static_cast<uint8_t>
        (static_cast<uint8_t>(can_msg.data[7]));


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

uint32_t DmMotor::double_to_raw(float value, float max, float min, uint8_t bit) const {
    double span = max - min;
    return (int) ((value-min)*((float)((1<<bit)-1))/span);
}

} // namespace meta_hardware
