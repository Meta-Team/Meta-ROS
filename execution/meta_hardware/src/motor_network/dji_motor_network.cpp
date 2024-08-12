#include <cmath>
#include <exception>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/can_driver/can_exceptions.hpp"
#include "meta_hardware/motor_driver/dji_motor_driver.hpp"
#include "meta_hardware/motor_network/dji_motor_network.hpp"

namespace meta_hardware {

DjiMotorNetwork::DjiMotorNetwork(
    std::string can_interface,
    const std::vector<std::unordered_map<std::string, std::string>> &motor_params) {
    std::vector<can_filter> can_filters;

    for (const auto &motor_param : motor_params) {
        std::string motor_model = motor_param.at("motor_model");
        uint32_t dji_motor_id = std::stoi(motor_param.at("motor_id"));

        auto dji_motor = std::make_shared<DjiMotor>(motor_model, dji_motor_id);
        motors_.emplace_back(dji_motor);

        rx_id2motor_[dji_motor->get_rx_can_id()] = dji_motor;

        // Initialize corresponding tx frame
        if (!tx_frames_.contains(dji_motor->get_tx_can_id())) {
            tx_frames_[dji_motor->get_tx_can_id()] = {
                .can_id = dji_motor->get_tx_can_id(), .len = 8, .data = {0}};
        }

        // Update the can filter
        can_filters.push_back(
            {.can_id = dji_motor->get_rx_can_id(), .can_mask = CAN_EFF_MASK});
    }

    // Initialize CAN driver
    can_driver_ = std::make_unique<CanDriver>(can_interface, false, can_filters);

    // Initialize RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

DjiMotorNetwork::~DjiMotorNetwork() {
    // Send zero effort to all motors
    for (canid_t tx_can_id : {0x1FE, 0x1FF, 0x200, 0x2FE, 0x2FF}) {
        if (tx_frames_.contains(tx_can_id)) {
            can_frame tx_frame{.can_id = tx_can_id, .len = 8, .data = {0}};
            try {
                can_driver_->write(tx_frame);
            } catch (CanIOException &e) {
                std::cerr << "Error writing CAN message: " << e.what() << std::endl;
            }
        }
    }
}

std::tuple<double, double, double> DjiMotorNetwork::read(size_t joint_id) const {
    return motors_[joint_id]->get_motor_feedback();
}

void DjiMotorNetwork::write(size_t joint_id, double effort) {
    const auto &motor = motors_[joint_id];
    uint32_t dji_motor_id = motor->get_dji_motor_id();
    uint32_t tx_can_id = motor->get_tx_can_id();

    uint32_t maximum_raw_effort = motor->get_maximum_raw_effort();
    effort = std::clamp(effort, -1.0, 1.0);
    auto effort_raw = static_cast<int16_t>(effort * maximum_raw_effort);

    can_frame &tx_frame = tx_frames_[tx_can_id];
    tx_frame.data[2 * ((dji_motor_id - 1) % 4)] = effort_raw >> 8;
    tx_frame.data[2 * ((dji_motor_id - 1) % 4) + 1] = effort_raw & 0xFF;
}

void DjiMotorNetwork::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            can_frame can_msg = can_driver_->read(2000);

            const auto &motor = rx_id2motor_.at(can_msg.can_id);

            auto position_raw =
                static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[0]) << 8) |
                                      static_cast<uint16_t>(can_msg.data[1]));
            auto velocity_raw =
                static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[2]) << 8) |
                                      static_cast<uint16_t>(can_msg.data[3]));
            auto current_raw =
                static_cast<uint16_t>((static_cast<uint16_t>(can_msg.data[4]) << 8) |
                                      static_cast<uint16_t>(can_msg.data[5]));

            motor->set_motor_feedback(position_raw, velocity_raw, current_raw);
        } catch (CanIOTimedOutException & /*e*/) {
            std::cerr << "Timed out waiting for DJI motor feedback." << std::endl;
        } catch (CanIOException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void DjiMotorNetwork::tx() {
    try {
        for (auto &frame_id : {0x1fe, 0x1ff, 0x200, 0x2fe, 0x2ff}) {
            if (tx_frames_.contains(frame_id)) {
                can_driver_->write(tx_frames_[frame_id]);
            }
        }
    } catch (CanIOException &e) {
        std::cerr << "Error writing CAN message: " << e.what() << std::endl;
    }
}

} // namespace meta_hardware
