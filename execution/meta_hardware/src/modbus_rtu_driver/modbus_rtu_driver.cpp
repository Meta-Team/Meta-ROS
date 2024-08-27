
#include "meta_hardware/modbus_rtu_driver/modbus_rtu_driver.hpp"
#include <cstdint>
#include <string>
#include <chrono>
#include <boost/crc.hpp>
#include <sys/types.h>
#include <iostream>
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

    std::string device_name = serial_params.at("device_name");
};


auto ModbusRtuDriver::awake_time()
{
    using std::chrono::operator""ms;
    return std::chrono::steady_clock::now() + 2ms;
}

void ModbusRtuDriver::set_command(int addr, int func, int reg, int len){
    command_[0] = static_cast<uint8_t>(addr);
    command_[1] = static_cast<uint8_t>(func);
    command_[2] = static_cast<uint8_t>(reg >> 8);
    command_[3] = static_cast<uint8_t>(reg & 0xFF);
    command_[4] = static_cast<uint8_t>(len >> 8);
    command_[5] = static_cast<uint8_t>(len & 0xFF);
    boost::crc_16_type crc_16;
    crc_16.process_bytes(command_.data(), 6);
    int crc = crc_16.checksum();
    command_[6] = static_cast<uint8_t>(crc & 0xFF);
    command_[7] = static_cast<uint8_t>(crc >> 8);
    
}

void ModbusRtuDriver::rx_tx_loop(std::stop_token stop_token){
    std::vector<uint8_t> address(1);
    std::vector<uint8_t> response(2);
    std::vector<uint8_t> data;
    std::vector<uint8_t> response_crc(2);
    std::vector<uint8_t> response_without_crc;
    while (!stop_token.stop_requested()) {
        std::this_thread::sleep_until(awake_time());

        // Send Command with blocking
        serial_driver_->port()->send(command_);
        // Receive Response with blocking

        response_without_crc.clear();
        serial_driver_->port()->receive(address);
        response_without_crc.emplace_back(address[0]);
        if( address[0] == command_[0]){
            serial_driver_->port()->receive(address);
            serial_driver_->port()->receive(response);
            uint8_t func_code = response[0];
            if(func_code != command_[1]){
                std::cout << " Function Code Mismatch. Expected: " 
                << command_[1] << " Received: " << func_code << std::endl;
                continue;
            }else if(func_code == 0x03){
                uint8_t byte_count = response[1];
                data.resize(byte_count);

                serial_driver_->port()->receive(data);

                response_without_crc.emplace_back(func_code);
                response_without_crc.emplace_back(byte_count);
                for(uint8_t i = 0; i < data.size(); i++){
                    response_without_crc.emplace_back(data[i]);
                }

                boost::crc_16_type crc_16;
                crc_16.process_bytes(response_without_crc.data(), response_without_crc.size());
                int calc_crc = crc_16.checksum();
                serial_driver_->port()->receive(response_crc);
                if((static_cast<int>(response_crc[0]) + (static_cast<int>(response_crc[1]) << 8))== calc_crc){
                    reg_data_ = data;
                }else{
                    std::cout << " CRC Mismatch. " << std::endl;
                }
            }else if(func_code == 0x10){
                std::vector<uint8_t> write_response(6);
                // get write response
                serial_driver_->port()->receive(write_response);
                // TODO: Check if the response is the same as the command
            }

        }else{
            std::cout << " Device Address Not Found. Device Address: " << address[0] << std::endl;
        }
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
