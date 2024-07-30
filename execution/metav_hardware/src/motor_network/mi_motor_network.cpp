#include <cstdlib>
#include <exception>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "CanMessage.hpp"
#include "exceptions/CanException.hpp"
#include "exceptions/CanInitException.hpp"
#include "metav_hardware/motor_driver/mi_motor_driver.hpp"
#include "metav_hardware/motor_network/mi_motor_network.hpp"

namespace metav_hardware {
using sockcanpp::CanDriver;
using sockcanpp::CanMessage;
using sockcanpp::exceptions::CanException;
using sockcanpp::exceptions::CanInitException;
using std::string;
using std::unordered_map;

MiMotorNetwork::MiMotorNetwork(const string &can_network_name, uint32_t host_id)
    : can_network_name_(can_network_name), host_id_(host_id) {
    // Initialize CAN driver
    try {
        can_driver_ = std::make_unique<CanDriver>(can_network_name_,
                                                  CanDriver::CAN_SOCK_RAW);
    } catch (CanInitException &e) {
        std::cerr << "Error initializing CAN driver: " << e.what() << std::endl;
        throw std::runtime_error("Error initializing CAN driver");
    }

    // Start RX thread
    rx_thread_ = std::thread(&MiMotorNetwork::rx_loop, this);

    // Enable all motors
    try {
        for (const auto &[joint_id, motor] : joint_id2motor_) {
            can_driver_->sendMessage(
                motor->get_motor_enable_frame(static_cast<uint8_t>(host_id_)));
        }
    } catch (CanException &e) {
        std::cerr << "Error writing MI motor enable CAN message: " << e.what()
                  << std::endl;
    }
}

MiMotorNetwork::~MiMotorNetwork() {
    // Disable all motors
    try {
        for (const auto &[joint_id, motor] : joint_id2motor_) {
            can_driver_->sendMessage(
                motor->get_motor_disable_frame(static_cast<uint8_t>(host_id_)));
        }
    } catch (CanException &e) {
        std::cerr << "Error writing MI motor disable CAN message: " << e.what()
                  << std::endl;
    }

    // TODO: Join RX thread
}

void MiMotorNetwork::add_motor(
    uint32_t joint_id, const unordered_map<string, string> &motor_params) {
    string motor_model = motor_params.at("motor_model");
    uint32_t mi_motor_id = std::stoi(motor_params.at("motor_id"));
    double Kp = std::stod(motor_params.at("Kp"));
    double Kd = std::stod(motor_params.at("Kd"));

    auto mi_motor = std::make_shared<MiMotor>(motor_model, mi_motor_id, Kp, Kd);
    motor_id2motor_[mi_motor_id] = mi_motor;
    joint_id2motor_[joint_id] = mi_motor;
}

std::tuple<double, double, double>
MiMotorNetwork::read(uint32_t joint_id) const {
    return joint_id2motor_.at(joint_id)->get_motor_feedback();
}

void MiMotorNetwork::write(uint32_t joint_id, double position, double velocity,
                           double effort) {
    const auto &motor = joint_id2motor_.at(joint_id);
    try {
        can_driver_->sendMessage(
            motor->get_motor_command_frame(position, velocity, effort));
    } catch (sockcanpp::exceptions::CanException &e) {
        std::cerr << "Error writing MI motor command CAN message: " << e.what()
                  << std::endl;
    }
}

[[noreturn]] void MiMotorNetwork::rx_loop() {
    while (true) {
        try {
            can_driver_->waitForMessages(std::chrono::milliseconds(200));
            CanMessage can_msg = can_driver_->readMessage();

            // MI motor frames are all extended frames
            if (!can_msg.getCanId().isExtendedFrameId()) {
                process_mi_frame(can_msg);
            }
        } catch (CanException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void MiMotorNetwork::process_mi_frame(const CanMessage &can_msg) {
    canid_t can_id = can_msg.getRawFrame().can_id & CAN_EFF_MASK;
    uint8_t mi_frame_type = can_id >> 24;
    switch (mi_frame_type) {
    case 0x00: // MI motor info frame
        process_mi_info_frame(can_msg);
        break;
    case 0x02: // MI motor feedback frame
        process_mi_fb_frame(can_msg);
        break;
    default:
        std::cerr << "Unknown MI motor frame type: " << mi_frame_type
                  << std::endl;
        break;
    }
}

void MiMotorNetwork::process_mi_info_frame(const CanMessage &can_msg) {
    // TODO: Implement this
}

void MiMotorNetwork::process_mi_fb_frame(const CanMessage &can_msg) {
    uint8_t motor_id = (can_msg.getRawFrame().can_id >> 8) & 0xFF;
    const auto &motor = motor_id2motor_.at(motor_id);
    motor->set_motor_feedback(can_msg);
}

void MiMotorNetwork::tx() {
    // tx() does nothing because the motor commands are sent in write()
}

} // namespace metav_hardware
