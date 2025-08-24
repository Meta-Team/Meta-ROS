#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <operation_interface/msg/dbus_control.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>

#define PUB_RATE 15 // ms

class ArmorTester : public rclcpp::Node
{
public:
    ArmorTester(const rclcpp::NodeOptions & options) : Node("armor_tester", options)
    {
        // reset
        ls_x = ls_y = rs_x = rs_y = wheel = 0;
        lsw = rsw = "";
        motor_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10,
            std::bind(&ArmorTester::dbus_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "Armor tester initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double ls_x, ls_y, rs_x, rs_y, wheel;
    std::string lsw, rsw;


    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        ls_x = std::clamp(msg->ls_x, -1.0, 1.0);
        ls_y = std::clamp(msg->ls_y, -1.0, 1.0);
        rs_x = std::clamp(msg->rs_x, -1.0, 1.0);
        rs_y = std::clamp(msg->rs_y, -1.0, 1.0);
        wheel = std::clamp(msg->wheel, -1.0, 1.0);
        lsw = msg->lsw;
        rsw = msg->rsw;
    }

    void timer_callback()
    {
        if (lsw != "MID") return;

        std_msgs::msg::Float64MultiArray tmp_motor_velocity;
        std::vector<double> motor_velocity;
        motor_velocity.push_back(ls_x*6.28*2);
        RCLCPP_INFO(this->get_logger(), "speed:%lf", ls_x*6.28*2);
        tmp_motor_velocity.data = motor_velocity;

        motor_pub_->publish(tmp_motor_velocity);
    }
};
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorTester)