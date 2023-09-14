#include "rclcpp/rclcpp.hpp"
#include <rclcpp/subscription.hpp>

#include "serial_output/serial.hpp"
#include "text_interface/msg/output_text.hpp"

class SerialOutput : public rclcpp::Node
{
private:
    Serial* serial_;
    rclcpp::Subscription<text_interface::msg::OutputText>::SharedPtr sub_;
    
    void msg_callback(text_interface::msg::OutputText msg)
    {
        serial_->send(msg.text);
    }

public:
    SerialOutput() : Node("serial_output")
    {
        serial_ = new Serial();
        sub_ = this->create_subscription<text_interface::msg::OutputText>(
            "output_text", 10, [this](text_interface::msg::OutputText::SharedPtr msg){
                this->msg_callback(*msg);
            });
    }

    ~SerialOutput()
    {
        delete serial_;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialOutput>());
    rclcpp::shutdown();
    return 0;
}