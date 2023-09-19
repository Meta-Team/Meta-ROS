#ifndef TELEOP_KEYBOARD_H
#define TELEOP_KEYBOARD_H

#include "rclcpp/rclcpp.hpp"
#include <termios.h>

#include "movement_interface/msg/natural_move.hpp"

class TeleopKeyboard : public rclcpp::Node
{
public:
    TeleopKeyboard();
    ~TeleopKeyboard();
    void display_menu();
    void set_goal(const char &ch);
    void disableWaitingForEnter();
    void restoreTerminalSettings();
    void publish_goal();

private:
    rclcpp::Publisher<movement_interface::msg::NaturalMove>::SharedPtr pub_;
    movement_interface::msg::NaturalMove nat_goal;
    termios oldt_;
};

#endif // TELEOP_KEYBOARD_H