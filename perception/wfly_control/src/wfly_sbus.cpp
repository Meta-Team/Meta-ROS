#include "wfly_control/wfly_sbus.hpp"
#include "CSerialPort/SerialPort_global.h"

#include <cstdint>
#include <fcntl.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>

using namespace itas109;

constexpr uint32_t sbus_baud_rate = 100000;
constexpr Parity sbus_parity = itas109::ParityEven;
constexpr DataBits sbus_data_bits = itas109::DataBits8;
constexpr StopBits sbus_stopbits = itas109::StopTwo;
constexpr FlowControl sbus_flow_control = itas109::FlowHardware;

WflySbus::WflySbus(std::string dev_path) : dev_path_(dev_path)
{
  serial_port_ = std::make_unique<CSerialPort>();
  serial_port_->init(
    dev_path_.c_str(),
    sbus_baud_rate,
    sbus_parity,
    sbus_data_bits,
    sbus_stopbits,
    sbus_flow_control
  );

  serial_port_->open();

  // Start rx thread
  rx_thread_ = std::make_unique<std::jthread>([this](std::stop_token s) { rx_loop(s); });
}

WflySbus::~WflySbus()
{
}

operation_interface::msg::WflyControl WflySbus::controller_msg()
{
  return wfly_msg_;
}

void WflySbus::rx_loop(std::stop_token stop_token) {
  uint8_t packet_buffer[25]; // Buffer for the complete packet
  
  while (!stop_token.stop_requested()) {

    if (!read_with_timeout(&packet_buffer[0], 1, 1000)) {
      continue;
    }

    if (packet_buffer[0] != 0x0F) {
      std::cout << "Skipping invalid header: 0x" << std::hex << static_cast<int>(packet_buffer[0]) << std::endl;
      continue;
    }

    if (!read_with_timeout(&packet_buffer[1], 24, 1000)) {
      std::cerr << "Failed to receive complete packet data" << std::endl;
      continue;
    }

    process_packet(packet_buffer);
  }
}

bool WflySbus::read_with_timeout(uint8_t* buffer, size_t bytes_to_read, int timeout_ms) {
  const auto start_time = std::chrono::steady_clock::now();
  size_t bytes_received = 0;

  while (bytes_received < bytes_to_read) {
    // Check for timeout
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time).count();
    if (elapsed > timeout_ms) {
      std::cerr << "Timeout after " << elapsed << " ms, received " 
      << bytes_received << "/" << bytes_to_read << " bytes" << std::endl;
      return false;
    }

    // Try to read remaining bytes
    int result = serial_port_->readData(buffer + bytes_received, bytes_to_read - bytes_received);

    if (result > 0) {
      bytes_received += result;
      // if (bytes_to_read > 1) {  // Only log for multi-byte reads
      //   std::cout << "Received " << result << " bytes, total: " 
      //   << bytes_received << "/" << bytes_to_read << std::endl;
      // }
    } else if (result < 0) {
      std::cerr << "Error reading from serial port: " << result << std::endl;
      return false;  // Return on error
    } else {
      // No data available, wait a bit before retrying
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  return true;  // Successfully read all bytes
}

void WflySbus::process_packet(const uint8_t* data) {
  if (!first_packet_arrived_) {
    first_packet_arrived_ = true;
  }

  SbusFrame* sbus_frame = reinterpret_cast<SbusFrame*>(const_cast<uint8_t*>(data));

  wfly_msg_.ls_x = (sbus_frame->ch3 - 0x400) / static_cast<double>(0x29e);
  wfly_msg_.ls_y = (sbus_frame->ch4 - 0x400) / static_cast<double>(0x29e);
  wfly_msg_.rs_x = (sbus_frame->ch2 - 0x400) / static_cast<double>(0x29e);
  wfly_msg_.rs_y = (sbus_frame->ch1 - 0x400) / static_cast<double>(0x29e);
  switch (sbus_frame->ch5) {
    case 0x69e:
      wfly_msg_.sa = "DOWN";
      break;
    case 0x161:
      wfly_msg_.sa = "UP";
      break;
    default:
      wfly_msg_.sa = "UP";
      break;
  }
  switch (sbus_frame->ch6) {
    case 0x69e:
      wfly_msg_.sb = "DOWN";
      break;
    case 0x400:
      wfly_msg_.sb = "MID";
      break;
    case 0x161:
      wfly_msg_.sb = "UP";
      break;
    default:
      wfly_msg_.sb = "UP";
      break;
  }
  switch (sbus_frame->ch7) {
    case 0x69e:
      wfly_msg_.sc = "DOWN";
      break;
    case 0x400:
      wfly_msg_.sc = "MID";
      break;
    case 0x161:
      wfly_msg_.sc = "UP";
      break;
    default:
      wfly_msg_.sc = "UP";
      break;
  }
  switch (sbus_frame->ch8) {
    case 0x69e:
      wfly_msg_.sd = "DOWN";
      break;
    case 0x161:
      wfly_msg_.sd = "UP";
      break;
    default:
      wfly_msg_.sd = "UP";
      break;
  }

  // Print the received data
  // std::cout << "Received SBUS data:" << std::endl;
  // std::cout << "ls_x: " << wfly_msg_.ls_x << std::endl;
  // std::cout << "ls_y: " << wfly_msg_.ls_y << std::endl;
  // std::cout << "rs_x: " << wfly_msg_.rs_x << std::endl;
  // std::cout << "rs_y: " << wfly_msg_.rs_y << std::endl;
  // std::cout << "sa: " << wfly_msg_.sa << std::endl;
  // std::cout << "sb: " << wfly_msg_.sb << std::endl;
  // std::cout << "sc: " << wfly_msg_.sc << std::endl;
  // std::cout << "sd: " << wfly_msg_.sd << std::endl;
  // std::cout << "======" << std::endl;
}
