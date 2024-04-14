#include "rclcpp/rclcpp.hpp"
#include "joy_arm/joy_interpreter.hpp"

class JoyArm : public rclcpp::Node
{
public:
    JoyArm() : Node("joy_arm")
    {
        // get param
        double linear_vel = this->declare_parameter("control.end_linear_vel", 0.1);
        double angular_vel = this->declare_parameter("control.end_angular_vel", 0.1);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        RCLCPP_INFO(this->get_logger(), "linear_vel: %f, angular_vel: %f, deadzone: %f",
            linear_vel, angular_vel, deadzone);

        interpreter_ = std::make_unique<JoyInterpreter>(linear_vel, angular_vel, deadzone);

        // pub and sub
        end_vel_pub_ = this->create_publisher<behavior_interface::msg::EndVel>("end_vel", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&JoyArm::joy_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PERIOD), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "JoyArm initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<JoyInterpreter> interpreter_;
    rclcpp::Publisher<behavior_interface::msg::EndVel>::SharedPtr end_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        interpreter_->input(msg);
    }

    void timer_callback()
    {
        interpreter_->update();
        end_vel_pub_->publish(*interpreter_->get_end_vel());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<JoyArm>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}