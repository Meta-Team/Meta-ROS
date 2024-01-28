#include "remote_control/remote.hpp"

std::unique_ptr<UartDriver> Remote::uart_ = std::make_unique<UartDriver>();

Remote::Remote()
{
    msg = std::make_unique<operation_interface::msg::RemoteControl>();
}

void Remote::get_frame()
{
    uart_->read(frame);
}

void Remote::process_rx()
{
    // TODO: implement this
}