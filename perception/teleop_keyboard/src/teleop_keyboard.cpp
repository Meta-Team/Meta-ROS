#include "teleop_keyboard/teleop_keyboard.h"

TeleopKeyboard::TeleopKeyboard() : Node("TeleopKeyboard")
{
    pub_ = this->create_publisher<movement_interface::msg::NaturalMove>("natural_move", 10);
    disableWaitingForEnter();
}

TeleopKeyboard::~TeleopKeyboard()
{
    restoreTerminalSettings();
    RCLCPP_INFO(this->get_logger(), "Terminating TeleopKeyboard");
    rclcpp::shutdown();
}

void TeleopKeyboard::display_menu()
{
    printf("Press a key to move the chassis:\n");
    printf("w: forward\n");
    printf("a: left\n");
    printf("s: backward\n");
    printf("d: right\n");
    printf("q: rotate counterclockwise\n");
    printf("e: rotate clockwise\n");
    printf("z: quit\n");
}

void TeleopKeyboard::disableWaitingForEnter()
{
    termios newt;

    tcgetattr(0, &oldt_);  /* Save terminal settings */
    newt = oldt_;  /* Init new settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

void TeleopKeyboard::restoreTerminalSettings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void TeleopKeyboard::set_goal(const char &ch)
{
    move_goal.vel_n = 0.0;
    move_goal.vel_tau = 0.0;
    move_goal.omega = 0.0;
    if (ch == 'w' || ch == 'W') move_goal.vel_tau = 1.0;
    else if (ch == 'a' || ch == 'A') move_goal.vel_n = 0.5;
    else if (ch == 's' || ch == 'S') move_goal.vel_tau = -1.0;
    else if (ch == 'd' || ch == 'D') move_goal.vel_n = -0.5;
    else if (ch == 'q' || ch == 'Q') move_goal.omega = 1.0;
    else if (ch == 'e' || ch == 'E') move_goal.omega = -1.0;
    else RCLCPP_INFO(this->get_logger(), "Invalid key input");
    publish_goal();
}

void TeleopKeyboard::publish_goal()
{
    RCLCPP_INFO(this->get_logger(), "Publishing: vel_n: %f, vel_tau: %f, omega: %f", move_goal.vel_n, move_goal.vel_tau, move_goal.omega);
    pub_->publish(move_goal);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopKeyboard>();
    node->display_menu();
    char ch;

    while (rclcpp::ok() && (ch = std::getchar()) != 'z' && ch != 'Z')
    {
        node->set_goal(ch);
    }
    
    return 0;
}