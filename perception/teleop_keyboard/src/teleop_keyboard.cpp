#include "rclcpp/rclcpp.hpp"
#include <termios.h>

#include "operation_interface/msg/teleop_key.hpp"

class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard() : Node("teleop_keyboard")
    {
        disableWaitingForEnter();
        pub_ = this->create_publisher<operation_interface::msg::TeleopKey>("teleop_key", 10);
    }

    ~TeleopKeyboard()
    {
        restoreTerminalSettings();
        rclcpp::shutdown();
    }

    void disableWaitingForEnter()
    {
        termios newt;
        tcgetattr(0, &oldt_);  /* Save terminal settings */
        newt = oldt_;  /* Init new settings */
        newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
        tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
    }

    void restoreTerminalSettings()
    {
        tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
    }

    void publish(char ch)
    {
        clear_msg();
        switch (ch)
        {
        case 'w': msg_.w = true; break;
        case 'a': msg_.a = true; break;
        case 's': msg_.s = true; break;
        case 'd': msg_.d = true; break;
        case 'q': msg_.q = true; break;
        case 'e': msg_.e = true; break;
        case 'i': msg_.i = true; break;
        case 'j': msg_.j = true; break;
        case 'o': msg_.o = true; break;
        case 'p': msg_.p = true; break;
        case ' ': msg_.space = true; break;
        default: break;
        }
        pub_->publish(msg_);
    }

private:
    termios oldt_;
    rclcpp::Publisher<operation_interface::msg::TeleopKey>::SharedPtr pub_;
    operation_interface::msg::TeleopKey msg_;

    void clear_msg()
    {
        msg_.w = false;
        msg_.a = false;
        msg_.s = false;
        msg_.d = false;
        msg_.q = false;
        msg_.e = false;
        msg_.i = false;
        msg_.j = false;
        msg_.o = false;
        msg_.p = false;
        msg_.space = false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    char ch;
    RCLCPP_INFO(node->get_logger(), "Press z to quit");

    while (rclcpp::ok() && (ch = std::getchar()) != 'z' && ch != 'Z')
    {
        node->publish(ch);
    }

    return 0;
}