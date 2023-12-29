#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/subscription.hpp>

#include "serial_output/uart_driver.hpp"
#include "text_interface/msg/output_text.hpp"

class SerialOutput : public rclcpp::Node
{
private:
    std::unique_ptr<UartDriver> uart_driver_;
    rclcpp::Subscription<text_interface::msg::OutputText>::SharedPtr sub_;
    
    void msg_callback(text_interface::msg::OutputText msg)
    {
        uart_driver_->send(msg.text);
    }

public:
    SerialOutput() : Node("serial_output")
    {
        uart_driver_ = std::make_unique<UartDriver>();
        sub_ = this->create_subscription<text_interface::msg::OutputText>(
            "output_text", 10, [this](text_interface::msg::OutputText::SharedPtr msg){
                this->msg_callback(*msg);
            });
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialOutput>());
    rclcpp::shutdown();
    return 0;
}