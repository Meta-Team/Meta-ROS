#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/can_driver/can_exceptions.hpp"
#include "meta_hardware/mi_motor_interface.hpp"
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

        auto mi_motor = std::make_shared<MiMotor>(joint_param, host_id);
        motor_id2motor_[mi_motor_id] = mi_motor;
        mi_motors_.emplace_back(mi_motor);
    }

    // Initialize CAN driver
    try {
        can_driver_ = std::make_unique<CanDriver>(can_network_name);
    } catch (CanException &e) {
        std::cerr << "Failed to initialize CAN driver: " << e.what() << std::endl;
        throw std::runtime_error("Failed to initialize MI motor network");
    }

    // Enable all motors
    for (const auto &motor : mi_motors_) {
        try {
            can_driver_->write(motor->get_motor_runmode_frame());
            can_driver_->write(motor->motor_enable_frame());
        } catch (CanIOException &e) {
            std::cerr << "Error writing MI motor enable CAN message: " << e.what()
                      << std::endl;
            throw std::runtime_error("Failed to enable MI motors");
        }
    }

    // Start RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

MiMotorNetwork::~MiMotorNetwork() {
    // Disable all motors
    try {
        for (const auto &motor : mi_motors_) {
            can_driver_->write(motor->motor_disable_frame());
        }
    } catch (CanIOException &e) {
        std::cerr << "Error writing MI motor disable CAN message: " << e.what()
                  << std::endl;
    }
}

std::tuple<double, double, double> MiMotorNetwork::read(uint32_t joint_id) const {
    return mi_motors_[joint_id]->get_motor_feedback();
}

void MiMotorNetwork::write_dyn(uint32_t joint_id, double position, double velocity,
                               double effort) {
    const auto &motor = mi_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_dyn_frame(position, velocity, effort));
    } catch (CanIOException &e) {
        std::cerr << "Error writing MI motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void MiMotorNetwork::write_pos(uint32_t joint_id, double position) {
    const auto &motor = mi_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_pos_frame(position));
    } catch (CanIOException &e) {
        std::cerr << "Error writing MI motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void MiMotorNetwork::write_vel(uint32_t joint_id, double velocity) {
    const auto &motor = mi_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_vel_frame(velocity));
    } catch (CanIOException &e) {
        std::cerr << "Error writing MI motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void MiMotorNetwork::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            auto can_msg = std::bit_cast<mi_can_frame>(can_driver_->read(2000));

            // MI motor frames are all extended frames
            if (can_msg.can_id.eff) {
                process_mi_frame(can_msg);
            }
        } catch (CanIOTimedOutException & /*e*/) {
            std::cerr << "Timed out waiting for MI motor feedback." << std::endl;
        } catch (CanIOException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

void MiMotorNetwork::process_mi_frame(const mi_can_frame &can_msg) {
    switch (can_msg.can_id.mode) {
    case 0x00: // MI motor info frame
        process_mi_info_frame(can_msg);
        break;
    case 0x02: // MI motor feedback frame
        process_mi_fb_frame(can_msg);
        break;
    default:
        std::cerr << "Unknown MI motor frame mode: " << can_msg.can_id.mode << std::endl;
        break;
    }
}

void MiMotorNetwork::process_mi_info_frame(const mi_can_frame &can_msg) {
    // TODO: Implement this
}

void MiMotorNetwork::process_mi_fb_frame(const mi_can_frame &can_msg) {
    uint8_t motor_id = can_msg.can_id.data & 0xFF;
    const auto &motor = motor_id2motor_.at(motor_id);
    motor->set_motor_feedback(can_msg);
}

} // namespace meta_hardware
