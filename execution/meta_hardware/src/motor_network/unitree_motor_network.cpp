#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <thread>
#include <chrono>

#include "meta_hardware/motor_network/unitree_motor_network.hpp"

namespace meta_hardware {

UnitreeMotorNetwork::UnitreeMotorNetwork(const std::string &tty_devpath,
                                         const std::vector<std::unordered_map<std::string, std::string>> &joint_params) {

    // Initialize Serial Port
    serial_port_ = std::make_unique<SerialPort>(tty_devpath);

    // Resize vectors for bulk communication
    size_t num_motors = joint_params.size();
    send_cmds_.resize(num_motors);
    recv_datas_.resize(num_motors);

    // Create Motor Drivers
    for (size_t i = 0; i < num_motors; ++i) {
        auto motor = std::make_shared<UnitreeMotor>(joint_params[i]);
        unitree_motors_.push_back(motor);
        
        // Initialize the command vector with default values (mode, ID, etc) from the driver
        motor->initialize_cmd(send_cmds_[i], recv_datas_[i]);
    }
}

UnitreeMotorNetwork::~UnitreeMotorNetwork() {
    // Optional: Send disable commands if needed
}

void UnitreeMotorNetwork::sync_read_write() {
    // TODO think of a better way to do this later
    // Iterate through each motor and perform the send/recv transaction sequentially
    for (size_t i = 0; i < send_cmds_.size(); ++i) {
        // Use the single-struct overload: sendRecv(MotorCmd*, MotorData*)
        bool success = serial_port_->sendRecv(&send_cmds_[i], &recv_datas_[i]);
        
        if (!success) {
            // Handle error (e.g., log warning)
            // std::cerr << "Unitree SendRecv failed for motor " << (int)send_cmds_[i].id << std::endl;
        }
    }
}

std::tuple<double, double, double> UnitreeMotorNetwork::read_state(uint32_t joint_index) const {
    if (joint_index >= unitree_motors_.size()) {
        throw std::out_of_range("Joint index out of range in read_state");
    }
    
    // Parse the received data using the driver
    return unitree_motors_[joint_index]->get_motor_feedback(recv_datas_[joint_index]);
}

void UnitreeMotorNetwork::write_command(uint32_t joint_index, double position, double velocity, double effort) {
    if (joint_index >= unitree_motors_.size()) {
        throw std::out_of_range("Joint index out of range in write_command");
    }

    // Update the specific command struct in the vector
    unitree_motors_[joint_index]->set_motor_command(send_cmds_[joint_index], position, velocity, effort);
}

} // namespace meta_hardware