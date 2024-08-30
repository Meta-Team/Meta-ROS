
#include "meta_hardware/modbus_rtu_driver/modbus_rtu_driver.hpp"
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>
#include <chrono>
#include <boost/crc.hpp>
#include <sys/types.h>
#include <iostream>
#include <thread>
#include <vector>

ModbusRtuDriver::ModbusRtuDriver(std::unordered_map<std::string, std::string> serial_params): 
    owned_ctx_{new IoContext(2)},
    serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {

    command_.resize(8);

    read_params(serial_params);
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>
    (baud_rate_, flow_control_, parity_, stop_bits_);

    try {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open()) {
            serial_driver_->port()->open();
            receive_thread_ = std::make_unique<std::jthread>([this](std::stop_token s) { rx_tx_loop(s); });
        }
    } catch (const std::exception & ex) {
        std::cerr << "Error creating serial port: " << device_name_ << " - " << ex.what() << std::endl;
        throw ex;
    }

}

void ModbusRtuDriver::read_params(std::unordered_map<std::string, std::string> serial_params){

    baud_rate_ = std::stoi(serial_params.at("baud_rate"));

    std::string fc = serial_params.at("flow_control");
    if (fc == "none") {
        flow_control_ = drivers::serial_driver::FlowControl::NONE;
    } else if (fc == "hardware") {
        flow_control_ = drivers::serial_driver::FlowControl::HARDWARE;
    } else if (fc == "software") {
        flow_control_ = drivers::serial_driver::FlowControl::SOFTWARE;
    } else {
        throw std::runtime_error("Unknown flow control: " + fc);
    }
    
    std::string pt = serial_params.at("parity");
    if (pt == "none") {
        parity_ = drivers::serial_driver::Parity::NONE;
    } else if (pt == "odd") {
        parity_ = drivers::serial_driver::Parity::ODD;
    } else if (pt == "even") {
        parity_ = drivers::serial_driver::Parity::EVEN;
    } else {
        throw std::runtime_error("Unknown parity: " + pt);
    }

    float sb = std::stof(serial_params.at("stop_bits"));
    if (sb == 1.0) {
        stop_bits_ = drivers::serial_driver::StopBits::ONE;
    } else if (sb == 1.5) {
        stop_bits_ = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
    } else if (sb == 2.0) {
        stop_bits_ = drivers::serial_driver::StopBits::TWO;
    } else {
        throw std::runtime_error("Unknown stop bits(Please Use Floating Point Notation): " + std::to_string(sb));
    }

    device_name_ = serial_params.at("device_name");
};


auto ModbusRtuDriver::awake_time()
{
    using std::chrono::operator""ms;
    return std::chrono::steady_clock::now() + 10ms;
}

void ModbusRtuDriver::set_command(int addr, int func, int reg, int len){
    command_[0] = static_cast<uint8_t>(addr);
    command_[1] = static_cast<uint8_t>(func);
    command_[2] = static_cast<uint8_t>(reg >> 8);
    command_[3] = static_cast<uint8_t>(reg & 0xFF);
    command_[4] = static_cast<uint8_t>(len >> 8);
    command_[5] = static_cast<uint8_t>(len & 0xFF);
    boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc_16;
    crc_16.process_bytes(command_.data(), 6);
    int crc = crc_16.checksum();
    command_[6] = static_cast<uint8_t>(crc & 0xFF);
    command_[7] = static_cast<uint8_t>(crc >> 8);
    reg_data_.resize(len);
    
}

void ModbusRtuDriver::rx_tx_loop(std::stop_token stop_token){
    int reg_num = static_cast<int>(command_[4] << 8 | command_[5]);
    int response_size = reg_num * 2 + 5;
    
    std::cout << "Reponse Size:" << response_size << std::endl;

    while (!stop_token.stop_requested()) {
        auto start_time = std::chrono::steady_clock::now(); 

        // std::cout << "Sending Command..." << std::endl;
        // Send Command with blocking
        serial_driver_->port()->send(command_);
        

        using std::chrono::operator""ms;
        std::this_thread::sleep_for(100ms);

        std::vector<uint8_t> response(response_size);
        serial_driver_->port()->receive(response);

        if(response[0] == command_[0] && response[1] == command_[1] && response[2] == (uint8_t)reg_num*2){
            // TODO: Only Implementing Reading Mode; Wrting Mode Not Implemented.
            for(int i = 0; i < reg_num; i++){
                reg_data_[i] = static_cast<int>(response[3 + i * 2] << 8 | response[4 + i * 2]);
            }
            boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> crc_16;
            crc_16.process_bytes(response.data(), response_size - 2);
            int crc = crc_16.checksum();
            if((crc & 0xFF) != response[response_size - 2] || (crc >> 8) != response[response_size - 1]){
                std::cout << "CRC Checked Failure. " << std::endl;
            }
        }else{
            std::cout << "Feedback Format Error." << std::endl;
        }
        // for(size_t i = 0; i < response.size(); i++){
        //     std::cout << std::hex << static_cast<int>(response[i]) << " ";
        // }
        // std::cout << std::endl;

        std::this_thread::sleep_until(start_time + 250ms);
    }
}


ModbusRtuDriver::~ModbusRtuDriver(){
        if (receive_thread_->joinable()) {
            receive_thread_->join();
        }

        if (serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }

        if (owned_ctx_) {
            owned_ctx_->waitForExit();
        }
    }
