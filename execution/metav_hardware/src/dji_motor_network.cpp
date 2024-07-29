#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "CanMessage.hpp"
#include "exceptions/CanException.hpp"
#include "exceptions/CanInitException.hpp"
#include "metav_hardware/motor_driver/dji_motor_driver.hpp"
#include "metav_hardware/motor_network/dji_motor_network.hpp"

namespace metav_hardware {
using sockcanpp::CanDriver;

DjiMotorNetwork::DjiMotorNetwork(std::string can_network_name) {
    try {
        can_driver_ = std::make_unique<CanDriver>(can_network_name,
                                                  CanDriver::CAN_SOCK_RAW);
    } catch (sockcanpp::exceptions::CanInitException &e) {
        std::cerr << "Error initializing CAN driver: " << e.what() << std::endl;
        throw std::runtime_error("Error initializing CAN driver");
    }
}

void DjiMotorNetwork::add_motor(std::string motor_model, uint32_t dji_motor_id,
                                uint32_t joint_id) {
    auto dji_motor = std::make_shared<DjiMotor>(motor_model, dji_motor_id);
    rx_id2motor_[dji_motor->get_rx_can_id()] = dji_motor;
    joint_id2motor_[joint_id] = dji_motor;
}

void DjiMotorNetwork::init_rx() {
    rx_thread_ = std::thread(&DjiMotorNetwork::rx_loop, this);
}

std::tuple<double, double, double>
DjiMotorNetwork::read(uint32_t joint_id) const {
    return joint_id2motor_.at(joint_id)->get_motor_feedback();
}

void DjiMotorNetwork::write(uint32_t joint_id, double effort) {
    const auto &motor = joint_id2motor_.at(joint_id);
    uint32_t dji_motor_id = motor->get_dji_motor_id();
    uint32_t tx_can_id = motor->get_tx_can_id();
    uint32_t maximum_raw_effort = motor->get_maximum_raw_effort();
    auto effort_raw = static_cast<int16_t>(effort * maximum_raw_effort);
    can_frame *tx_frame = nullptr;
    switch (tx_can_id) {
    case 0x1FE:
        tx_frame = &tx_frame_1fe;
        break;
    case 0x1FF:
        tx_frame = &tx_frame_1ff;
        break;
    case 0x200:
        tx_frame = &tx_frame_200;
        break;
    case 0x2FE:
        tx_frame = &tx_frame_2fe;
        break;
    case 0x2FF:
        tx_frame = &tx_frame_2ff;
        break;
    default:
        throw std::runtime_error("Unknown tx CAN ID: " +
                                 std::to_string(tx_can_id));
        return;
    }
    tx_frame->data[2 * ((dji_motor_id - 1) % 4)] = effort_raw >> 8;
    tx_frame->data[2 * ((dji_motor_id - 1) % 4) + 1] = effort_raw & 0xFF;
}

[[noreturn]] void DjiMotorNetwork::rx_loop() {
    while (true) {
        try {
            can_driver_->waitForMessages(std::chrono::milliseconds(200));
            sockcanpp::CanMessage can_msg = can_driver_->readMessage();
            const auto &motor = rx_id2motor_.at(can_msg.getRawFrame().can_id);

            auto position_raw = static_cast<uint16_t>(
                (static_cast<uint16_t>(can_msg.getRawFrame().data[0]) << 8) |
                static_cast<uint16_t>(can_msg.getRawFrame().data[1]));
            auto velocity_raw = static_cast<uint16_t>(
                (static_cast<uint16_t>(can_msg.getRawFrame().data[2]) << 8) |
                static_cast<uint16_t>(can_msg.getRawFrame().data[3]));
            auto current_raw = static_cast<uint16_t>(
                (static_cast<uint16_t>(can_msg.getRawFrame().data[4]) << 8) |
                static_cast<uint16_t>(can_msg.getRawFrame().data[5]));

            motor->set_motor_feedback(position_raw, velocity_raw, current_raw);
        } catch (sockcanpp::exceptions::CanException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void DjiMotorNetwork::tx() {
    try {
        // can_driver_->sendMessage(tx_frame_1fe);
        can_driver_->sendMessage(tx_frame_1ff);
        can_driver_->sendMessage(tx_frame_200);
        // can_driver_->sendMessage(tx_frame_2fe);
        can_driver_->sendMessage(tx_frame_2ff);
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
    } catch (sockcanpp::exceptions::CanException &e) {
        std::cerr << "Error writing CAN message: " << e.what() << std::endl;
    }
}

} // namespace metav_hardware
