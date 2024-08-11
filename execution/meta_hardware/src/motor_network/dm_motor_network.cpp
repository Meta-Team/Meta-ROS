#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/motor_driver/dm_motor_driver.hpp"
#include "meta_hardware/motor_network/dm_motor_network.hpp"

namespace meta_hardware {

DmMotorNetwork::DmMotorNetwork(
    std::string can_interface,
    uint32_t master_id,
    const std::vector<std::unordered_map<std::string, std::string>>
        &motor_params) : master_id_(master_id) {
    std::vector<can_filter> can_filters;

    for (const auto &motor_param : motor_params) {
        std::string motor_model = motor_param.at("motor_model");
        uint32_t dm_motor_id = std::stoi(motor_param.at("motor_id"));
        std::string motor_mode = motor_param.at("motor_mode");
        double max_vel = std::stod(motor_param.at("max_vel"));
        double max_pos = std::stod(motor_param.at("max_pos"));
        double max_torque = std::stod(motor_param.at("max_torque"));


        auto dm_motor = 
            std::make_shared<DmMotor>(motor_model, dm_motor_id, motor_mode, max_vel, max_pos, max_torque);
        motors_.emplace_back(dm_motor);

        motor_id2motor_[dm_motor_id] = dm_motor;

    }

    can_filters.push_back(
            {.can_id = master_id, .can_mask = CAN_SFF_MASK});

    // Initialize CAN driver
    can_driver_ =
        std::make_unique<CanDriver>(can_interface, false, can_filters);

    // Initialize RX thread
    rx_thread_ =
        std::make_unique<std::jthread>(&DmMotorNetwork::rx_loop, this);
}

DmMotorNetwork::~DmMotorNetwork() {
    // Send zero effort to all motors
    for (canid_t tx_can_id : {0x1FE, 0x1FF, 0x200, 0x2FE, 0x2FF}) {
        can_frame tx_frame{.can_id = tx_can_id, .len = 8, .data = {0}};
        can_driver_->write(tx_frame);
    }

    // Stop the RX thread
    rx_thread_running_ = false;
}

std::tuple<double, double, double>
DmMotorNetwork::read(uint32_t joint_id) const {
    return motors_[joint_id]->get_motor_feedback();
}

void DmMotorNetwork::write(uint32_t joint_id, double effort) {
    const auto &motor = motors_[joint_id];
    uint32_t dm_motor_id = motor->get_dm_motor_id();
    uint32_t tx_can_id = motor->get_tx_can_id();

    // uint32_t maximum_raw_effort = motor->get_maximum_raw_effort();
    // effort = std::clamp(effort, -1.0, 1.0);
    // auto effort_raw = static_cast<int16_t>(effort * maximum_raw_effort);

    // can_frame &tx_frame = tx_frames_[tx_can_id];
    // tx_frame.data[2 * ((dm_motor_id - 1) % 4)] = effort_raw >> 8;
    // tx_frame.data[2 * ((dm_motor_id - 1) % 4) + 1] = effort_raw & 0xFF;
}

void DmMotorNetwork::rx_loop() {
    while (rx_thread_running_) {
        try {
            can_frame can_msg = can_driver_->read();

            const auto &motor = motor_id2motor_.at(can_msg.can_id);

            auto error_code = static_cast<uint8_t>
                (static_cast<uint8_t>(can_msg.data[0]) >> 4);
            
            auto id = static_cast<uint8_t>
                ((static_cast<uint8_t>(can_msg.data[0]) & 0x0F));

            auto position_raw = static_cast<int32_t>
                (((static_cast<uint32_t>(can_msg.data[1]) & 0xFF) << 8) | 
                (static_cast<uint32_t>(can_msg.data[2]) & 0xFF) 
                );

            auto velocity_raw = static_cast<int32_t>
                (((static_cast<uint32_t>(can_msg.data[3]) & 0xFF) << 8) | 
                ((static_cast<uint32_t>(can_msg.data[4]) & 0xC0) >> 12) 
                );

            auto torque_raw = static_cast<int32_t>
                (((static_cast<uint32_t>(can_msg.data[4]) & 0x3F) << 4) | 
                (static_cast<uint32_t>(can_msg.data[5]) & 0xFF) 
                );

            auto temperature_mos = static_cast<uint8_t>
                (static_cast<uint8_t>(can_msg.data[6]));

            
            auto temperature_rotor = static_cast<uint8_t>
                (static_cast<uint8_t>(can_msg.data[7]));

            motor->set_motor_feedback(error_code, id, position_raw, velocity_raw, 
                                      torque_raw, temperature_mos, temperature_rotor);
        } catch (std::runtime_error &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void DmMotorNetwork::tx() {
    try {
        for (auto &frame_id : {0x1fe, 0x1ff, 0x200, 0x2fe, 0x2ff}) {
            if (tx_frames_.contains(frame_id)) {
                can_driver_->write(tx_frames_[frame_id]);
            }
        }
    } catch (std::runtime_error &e) {
        std::cerr << "Error writing CAN message: " << e.what() << std::endl;
    }
}

} // namespace meta_hardware
