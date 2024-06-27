#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include "wheel_leg_pid/wheel_leg.h"

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_goal.hpp"

class WheelLegPid : public rclcpp::Node
{
public:
    WheelLegPid() : Node("wheel_leg_pid")
    {
        wheel_leg_ = [this] {
            // omega_wv
            std::vector<double> params = {0, 0, 0};
            params = this->declare_parameter("wheel_leg.omega_wv", params);
            Param omega_wv_p(params[0], params[1], params[2]);
            // obli_wv
            params = {0, 0, 0};
            params = this->declare_parameter("wheel_leg.obli_wv", params);
            Param obli_wv_p(params[0], params[1], params[2]);
            // bv_obli
            params = {0, 0, 0};
            params = this->declare_parameter("wheel_leg.bv_obli", params);
            Param bv_obli_p(params[0], params[1], params[2]);

            return std::make_unique<WheelLeg>(omega_wv_p, obli_wv_p, bv_obli_p);
        }();

        pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
        sub_ = this->create_subscription<behavior_interface::msg::Move>(
            "move", 10,
            std::bind(&WheelLegPid::move_callback, this, std::placeholders::_1)
        );
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WheelLegPid::timer_callback, this)
        );
    }

private:
    void move_callback(const behavior_interface::msg::Move::SharedPtr msg)
    {
        wheel_leg_->set_goal(msg->vel_x, msg->omega);
    }

    void timer_callback()
    {
        pub_->publish(wheel_leg_->get_msg());
    }

    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr pub_;
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    // TODO: imu sub

    std::unique_ptr<WheelLeg> wheel_leg_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelLegPid>());
    rclcpp::shutdown();
    return 0;
}