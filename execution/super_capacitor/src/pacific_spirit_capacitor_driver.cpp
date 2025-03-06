#include <thread>
#include <iostream>
#include <exception>
#include <linux/can.h>
#include <stdexcept>
#include <string>

#include "meta_hardware/can_driver/can_driver.hpp"
#include "meta_hardware/can_driver/can_exceptions.hpp"
#include "super_capacitor/super_capacitor_base.h"
#include "super_capacitor/pacific_spirit_capacitor_driver.h"

namespace super_capacitor {
PacificSpiritCapacitorDriver::PacificSpiritCapacitorDriver() 
    : super_capacitor::SuperCapacitorBase(){}

void PacificSpiritCapacitorDriver::init(std::string can_interface){
    can_filters_.push_back({.can_id = 0x2c8, .can_mask = CAN_EFF_MASK});
    // Initialize CAN driver
    can_driver_ = std::make_unique<meta_hardware::CanDriver>(can_interface, false, can_filters_);

    // Initialize RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

void PacificSpiritCapacitorDriver::set_target_power(double power) {
    target_power_ = power;
}

void PacificSpiritCapacitorDriver::set_referee_power(double power) {
    referee_power_ = power;
}

std::unordered_map<std::string, double> PacificSpiritCapacitorDriver::get_state() {
    return {
        {"max_discharge_power", max_discharge_power_},
        {"base_power", base_power_},
        {"cap_energy_percentage", cap_energy_percentage_}, 
        {"cap_state", cap_state_}
    };
}

std::string PacificSpiritCapacitorDriver::get_device_name() {
    return "Xidi Capacitor";
}

void PacificSpiritCapacitorDriver::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            can_frame can_msg = can_driver_->read(2000);

            max_discharge_power_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[0]) |
                            (static_cast<uint16_t>(can_msg.data[1]) << 8)) / 100.0);
            base_power_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[2]) |
                            (static_cast<uint16_t>(can_msg.data[3]) << 8)) / 100.0);
            cap_energy_percentage_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[4]) |
                            (static_cast<uint16_t>(can_msg.data[5]) << 8)) / 100.0);
            cap_state_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[6]) |
                            (static_cast<uint16_t>(can_msg.data[7]) << 8)) / 100.0);
        } catch (const meta_hardware::CanIOException &e) {
            std::cerr << "Error reading super capacitor CAN message: " << e.what() << std::endl;
        }
    }

}

void PacificSpiritCapacitorDriver::tx(){
    can_frame tx_frame{.can_id = 0x2c7, .len = 8, .data = {0}};
    tx_frame.data[0] = static_cast<uint8_t>(static_cast<uint32_t>(target_power_ * 100.0) & 0xFF);
    tx_frame.data[1] = static_cast<uint8_t>(static_cast<uint32_t>(target_power_ * 100.0) >> 8);
    tx_frame.data[2] = static_cast<uint8_t>(static_cast<uint32_t>(referee_power_ * 100.0) & 0xFF);
    tx_frame.data[3] = static_cast<uint8_t>(static_cast<uint32_t>(referee_power_ * 100.0) >> 8);
    tx_frame.data[4] = 0x12;
    tx_frame.data[5] = 0x20;
    tx_frame.data[6] = 0x12;
    tx_frame.data[7] = 0x07;
    try {
        can_driver_->write(tx_frame);
    } catch (meta_hardware::CanIOException &e) {
        std::cerr << "Error writing CAN message: " << e.what() << std::endl;
    }
}

PacificSpiritCapacitorDriver::~PacificSpiritCapacitorDriver() {
    if (rx_thread_ && rx_thread_->joinable()) {
        rx_thread_->request_stop();  // Gracefully stop the thread
        rx_thread_->join();
    }
}

}

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(super_capacitor::PacificSpiritCapacitorDriver, super_capacitor::SuperCapacitorBase)