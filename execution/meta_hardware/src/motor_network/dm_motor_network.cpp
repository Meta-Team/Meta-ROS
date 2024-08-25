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

DmMotorNetwork::DmMotorNetwork(const std::string &can_network_name, uint8_t master_id,
                               const std::vector<std::unordered_map<std::string, std::string>> &joint_params)
        : master_id_(master_id) {

    std::vector<can_filter> can_filters;

    for (const auto &joint_param : joint_params) {
        std::string motor_model = joint_param.at("motor_model");
        uint32_t dm_motor_id = std::stoi(joint_param.at("motor_id"));
        std::string motor_mode = joint_param.at("control_mode");

        auto dm_motor = 
            std::make_shared<DmMotor>(joint_param, master_id);
        dm_motors_.emplace_back(dm_motor);

        motor_id2motor_[dm_motor_id] = dm_motor;

    }

    can_filters.push_back(
            {.can_id = master_id, .can_mask = CAN_SFF_MASK});

    // Initialize CAN driver
    can_driver_ =
        std::make_unique<CanDriver>(can_network_name, false, can_filters);

    // Enable all motors
    for (const auto &motor : dm_motors_) {
        can_driver_->write(motor->motor_enable_frame());
    }

    // Initialize RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

DmMotorNetwork::~DmMotorNetwork() {
    // Disable all motors
    try {
        for (const auto &motor : dm_motors_) {
            can_driver_->write(motor->motor_disable_frame());
        }
    } catch (CanIOException &e) {
        std::cerr << "Error writing DM motor disable CAN message: " << e.what()
                  << std::endl;
    }
}

std::tuple<double, double, double>
DmMotorNetwork::read(uint32_t joint_id) const {
    return dm_motors_[joint_id]->get_motor_feedback();
}

void DmMotorNetwork::write_mit(uint32_t joint_id, double position, double velocity, double effort){
    const auto &motor = dm_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_mit_frame(position, velocity, effort));
    } catch (CanIOException &e) {
        std::cerr << "Error writing DM motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void DmMotorNetwork::write_pos(uint32_t joint_id, double position){
    const auto &motor = dm_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_pos_frame(position));
    } catch (CanIOException &e) {
        std::cerr << "Error writing DM motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void DmMotorNetwork::write_vel(uint32_t joint_id, double velocity){
    const auto &motor = dm_motors_[joint_id];
    try {
        can_driver_->write(motor->motor_vel_frame(velocity));
    } catch (CanIOException &e) {
        std::cerr << "Error writing DM motor command CAN message: " << e.what()
                  << std::endl;
    }
}

void DmMotorNetwork::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            auto can_msg = can_driver_->read(2000);
            const auto &motor = motor_id2motor_.at(can_msg.data[0] & 0x0F);
            motor->set_motor_feedback(can_msg);
        } catch (CanIOTimedOutException & /*e*/) {
            std::cerr << "Timed out waiting for DM motor feedback." << std::endl;
        } catch (CanIOException &e) {
            std::cerr << "Error reading CAN message: " << e.what() << std::endl;
        }
    }
}

} // namespace meta_hardware
