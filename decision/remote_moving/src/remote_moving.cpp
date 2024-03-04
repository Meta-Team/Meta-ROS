#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/remote_control.hpp"
#include "behavior_interface/msg/move.hpp"

#define UNIT_VEL 500
#define RECOV_R 100
#define SEND_R 20

class RemoteMoving : public rclcpp::Node
{
public:
    RemoteMoving() : Node("remote_moving")
    {
        vel = std::make_unique<behavior_interface::msg::Move>();
        sub_ = this->create_subscription<operation_interface::msg::RemoteControl>(
            "remote_control", 10,
            std::bind(&RemoteMoving::control_callback, this, std::placeholders::_1));
        recov_timer_ = this->create_wall_timer(std::chrono::milliseconds(RECOV_R),
            std::bind(&RemoteMoving::recov_callback, this));
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(SEND_R),
            std::bind(&RemoteMoving::send_callback, this));
        pub_ = this->create_publisher<behavior_interface::msg::Move>("chassis_move", 10);
    }

private:
    rclcpp::Subscription<operation_interface::msg::RemoteControl>::SharedPtr sub_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr recov_timer_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    bool to_stop = true;
    behavior_interface::msg::Move::UniquePtr vel;

    void control_callback(const operation_interface::msg::RemoteControl::SharedPtr msg)
    {
        to_stop = false;
        reset_goal();
        if (msg->w) vel->vel_x += UNIT_VEL;
        if (msg->s) vel->vel_x -= UNIT_VEL;
        if (msg->a) vel->vel_y += UNIT_VEL;
        if (msg->d) vel->vel_y -= UNIT_VEL;
        if (msg->q) vel->omega += UNIT_VEL;
        if (msg->e) vel->omega -= UNIT_VEL;
    }

    void send_callback()
    {
        pub_->publish(std::move(vel));
    }

    void recov_callback()
    {
        // if to_stop is already true, stop
        if (to_stop == true) reset_goal();
        // whether to stop or not, set to_stop to true, which can be overwritten by control_callback
        to_stop = true;
    }

    void reset_goal()
    {
        vel->vel_x = 0;
        vel->vel_y = 0;
        vel->omega = 0;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteMoving>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}