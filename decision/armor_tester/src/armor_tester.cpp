#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <operation_interface/msg/dbus_control.hpp>
#include "behavior_interface/msg/armor.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>

#define PUB_RATE 15 // ms
#define PI 3.1415926

class ArmorTester : public rclcpp::Node
{
public:
    ArmorTester(const rclcpp::NodeOptions & options) : Node("armor_tester", options)
    {
        // reset
        ls_x = ls_y = rs_x = rs_y = wheel = 0;
        lsw = rsw = "";
        velocity_pub_ = this->create_publisher<behavior_interface::msg::Armor>(
            "/armor_tester_controller/reference", 10);

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
    rclcpp::Publisher<behavior_interface::msg::Armor>::SharedPtr velocity_pub_;
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

        behavior_interface::msg::Armor armor_tester_velocity;
        armor_tester_velocity.unitree_vel = ls_x * 2 * PI;
        armor_tester_velocity.dji_vel = rs_x * 2 * PI;

        RCLCPP_INFO(this->get_logger(), "unitree_vel:%lf, dji_vel: %lf",
                    ls_x * 2 * PI, rs_x * 2 * PI);
        velocity_pub_->publish(armor_tester_velocity);
    }
};
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ArmorTester)
