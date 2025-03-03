#include "xidi_capacitor_driver.h"

namespace super_capacitor_plugin{
XidiCapacitorDriver::XidiCapacitorDriver(std::string can_interface)
{
    can_filters.push_back({.can_id = 0x211, .can_mask = CAN_EFF_MASK});
    // Initialize CAN driver
    can_driver_ = std::make_unique<CanDriver>(can_interface, false, can_filters);

    // Initialize RX thread
    rx_thread_ =
        std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

void XidiCapacitorDriver::set_target_power(double power) {
    target_power_ = power;
}

void XidiCapacitorDriver::set_referee_power(double power) {
    referee_power_ = power;
}

std::unordered_map<std::string, double> XidiCapacitorDriver::get_state() {
    return {
        {"input_voltage", input_voltage_},
        {"capacitor_voltage", capacitor_voltage_},
        {"input_current", input_current_}, 
        {"target_power", target_power_}
    };
}

std::string XidiCapacitorDriver::get_device_name() {
    return "Xidi Capacitor";
}

void XidiCapacitorDriver::rx_loop(std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
        try {
            can_frame can_msg = can_driver_->read(2000);

            input_voltage_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[0]) |
                            (static_cast<uint16_t>(can_msg.data[1]) << 8)) / 100.0);
            capacitor_voltage_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[2]) |
                            (static_cast<uint16_t>(can_msg.data[3]) << 8)) / 100.0);
            input_current_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[0]) |
                            (static_cast<uint16_t>(can_msg.data[1]) << 8)) / 100.0);
            capacitor_current_ = static_cast<double>(static_cast<uint16_t>(static_cast<uint16_t>(can_msg.data[2]) |
                            (static_cast<uint16_t>(can_msg.data[3]) << 8)) / 100.0);
        } catch (const std::out_of_range &e) {
            std::cerr << "Unknown capacitor CAN ID: " << can_msg.can_id << std::endl;
        } catch (const CanIOException &e) {
            std::cerr << "Error reading super capacitor CAN message: " << e.what() << std::endl;
        }
    }

}

XidiCapacitorDriver::tx(){
    canframe tx_frame{.can_id = 0x210, .len = 2, .data = {0}};
    tx_frame.data[0] = static_cast<uint8_t>(static_cast<uint32_t>(target_power_ * 100.0) >> 8);
    tx_frame.data[1] = static_cast<uint8_t>(static_cast<uint32_t>(target_power_ * 100.0) & 0xFF);
    try {
        can_driver_->write(tx_frame);
    } catch (CanIOException &e) {
        std::cerr << "Error writing CAN message: " << e.what() << std::endl;
    }
}

XidiCapacitorDriver::~XidiCapacitorDriver() {
    return;
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(super_capacitor_plugins::XidiCapacitorDriver, super_capacitor_base::SuperCapacitor)