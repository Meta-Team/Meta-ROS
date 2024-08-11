#include <cstdlib>
#include <exception>
#include <iostream>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/motor_driver/mi_motor_driver.hpp"
#include "meta_hardware/motor_network/mi_motor_network.hpp"

namespace meta_hardware {
using std::string;
using std::unordered_map;
using std::vector;

MiMotorNetwork::MiMotorNetwork(const string &can_network_name, uint32_t host_id,
                               const vector<unordered_map<string, string>> &joint_params)
    : host_id_(host_id) {

    // Initialize MI motor drivers
    for (const auto &joint_param : joint_params) {
        string motor_model = joint_param.at("motor_model");
        uint32_t mi_motor_id = std::stoi(joint_param.at("motor_id"));
        double Kp = std::stod(joint_param.at("Kp"));
        double Kd = std::stod(joint_param.at("Kd"));

        auto mi_motor = std::make_shared<MiMotor>(motor_model, mi_motor_id, Kp, Kd);
        motor_id2motor_[mi_motor_id] = mi_motor;
        mi_motors_.emplace_back(mi_motor);
    }

    // Initialize CAN driver
    can_driver_ = std::make_unique<CanDriver>(can_network_name);

    // Enable all motors
    for (const auto &motor : mi_motors_) {
        can_driver_->write(motor->get_motor_enable_frame(static_cast<uint8_t>(host_id_)));
    }

    // Start RX thread
    rx_thread_ = std::thread(&MiMotorNetwork::rx_loop, this);
}

MiMotorNetwork::~MiMotorNetwork() {
    // Disable all motors
    try {
        for (const auto &motor : mi_motors_) {
            can_driver_->write(
                motor->get_motor_disable_frame(static_cast<uint8_t>(host_id_)));
        }
    } catch (std::runtime_error &e) {
        std::cerr << "Error writing MI motor disable CAN message: " << e.what()
                  << std::endl;
    }

    // TODO: Join RX thread
}

std::tuple<double, double, double> MiMotorNetwork::read(uint32_t joint_id) const {
    return mi_motors_[joint_id]->get_motor_feedback();
}

void MiMotorNetwork::write(uint32_t joint_id, double position, double velocity,
                           double effort) {
    const auto &motor = mi_motors_[joint_id];
    try {
        can_driver_->write(motor->get_motor_command_frame(position, velocity, effort));
    } catch (std::runtime_error &e) {
        std::cerr << "Error writing MI motor command CAN message: " << e.what()
                  << std::endl;
    }
}

[[noreturn]] void MiMotorNetwork::rx_loop() {
    while (true) {
        try {
            can_frame can_msg = can_driver_->read();

            // MI motor frames are all extended frames
            if (can_msg.can_id & CAN_EFF_FLAG) {
                process_mi_frame(can_msg);
            }
        } catch (std::runtime_error &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void MiMotorNetwork::process_mi_frame(const can_frame &can_msg) {
    canid_t can_id = can_msg.can_id & CAN_EFF_MASK;
    uint8_t mi_frame_type = can_id >> 24;
    switch (mi_frame_type) {
    case 0x00: // MI motor info frame
        process_mi_info_frame(can_msg);
        break;
    case 0x02: // MI motor feedback frame
        process_mi_fb_frame(can_msg);
        break;
    default:
        std::cerr << "Unknown MI motor frame type: " << mi_frame_type << std::endl;
        break;
    }
}

void MiMotorNetwork::process_mi_info_frame(const can_frame &can_msg) {
    // TODO: Implement this
}

void MiMotorNetwork::process_mi_fb_frame(const can_frame &can_msg) {
    uint8_t motor_id = (can_msg.can_id >> 8) & 0xFF;
    const auto &motor = motor_id2motor_.at(motor_id);
    motor->set_motor_feedback(can_msg);
}

} // namespace meta_hardware
