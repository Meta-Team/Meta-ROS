#include "remote_control/remote_control.hpp"
#include "remote_control/remote.hpp"

constexpr const char * RemoteControl::dev_name;
constexpr const char * RemoteControl::dev_null;
constexpr uint32_t RemoteControl::baud;
constexpr FlowControl RemoteControl::fc;
constexpr Parity RemoteControl::pt;
constexpr StopBits RemoteControl::sb;

RemoteControl::RemoteControl()
{
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    node_ = rclcpp::Node::make_shared("remote_control");
    
    if (!port_->is_open())
    {
        port_->open();
        receive_thread = std::thread(&RemoteControl::receive, this);
    }
}

RemoteControl::~RemoteControl()
{
    if (receive_thread.joinable())
    {
        receive_thread.join();
    }
    if (port_->is_open())
    {
        port_->close();
    }
    if (ctx_)
    {
        ctx_->waitForExit();
    }
}

void RemoteControl::receive()
{
    std::vector<uint8_t> header(7);
    std::vector<uint8_t> data;
    data.reserve(sizeof(Remote::RemoteFrame::Data));
}