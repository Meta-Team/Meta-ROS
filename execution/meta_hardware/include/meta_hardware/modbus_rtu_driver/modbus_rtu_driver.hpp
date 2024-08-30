#ifndef META_HARDWARE__MODBUS_RTU_DRIVER_HPP_
#define META_HARDWARE__MODBUS_RTU_DRIVER_HPP_

#include <cstdint>
#include <memory>
#include <serial_driver/serial_driver.hpp>
#include <serial_driver/serial_port.hpp>
#include <stop_token>
#include <thread>
#include <unordered_map>
#include <vector>


class ModbusRtuDriver
{
public:
    explicit ModbusRtuDriver(std::unordered_map<std::string, std::string> serial_params);
    ~ModbusRtuDriver();

    void set_command(int addr, int func, int reg, int len);
    std::vector<int> get_reg_data(){return reg_data_;}; 

private:
    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::unique_ptr<std::jthread> receive_thread_;

    int baud_rate_;
    drivers::serial_driver::FlowControl flow_control_;
    drivers::serial_driver::Parity parity_;
    drivers::serial_driver::StopBits stop_bits_;

    std::vector<uint8_t> command_;
    std::vector<int> reg_data_;
    
    void rx_tx_loop(std::stop_token stop_token);
    auto awake_time();
    void read_params(std::unordered_map<std::string, std::string> serial_params);
    void send_command(std::vector<uint8_t> data);
    
    

};
#endif // META_HARDWARE__MODBUS_RTU_DRIVER_HPP_