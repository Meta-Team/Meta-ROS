#include "rclcpp/rclcpp.hpp"

#include "shooting_interface/msg/shoot.hpp"
#include "motor_interface/msg/motor_goal.hpp"

enum MotorId
{
    FRIC_U = 5,
    FRIC_D = 6,
    FEED_L = 7,
    FEED_R = 8,
};

class ShootLoad : public rclcpp::Node
{
public:
    ShootLoad() : Node("shoot_load")
    {
        shoot_sub_ = this->create_subscription<shooting_interface::msg::Shoot>("shoot",
            10, std::bind(&ShootLoad::shoot_callback, this, std::placeholders::_1));
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        get_param();
        RCLCPP_INFO(this->get_logger(), "ShootLoad initialized.");
    }

private:
    rclcpp::Subscription<shooting_interface::msg::Shoot>::SharedPtr shoot_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;

    float feed_vel_r = 300.0, fric_vel_u = 300.0, fric_vel_d = 300.0;

    void get_param()
    {
        feed_vel_r = this->declare_parameter("gimbal.feed_vel_r", feed_vel_r);
        fric_vel_u = this->declare_parameter("gimbal.fric_vel_u", fric_vel_u);
        fric_vel_d = this->declare_parameter("gimbal.fric_vel_d", fric_vel_d);
        RCLCPP_INFO(this->get_logger(), "Feed velocity: %f, Friction velocity: %f and %f", feed_vel_r, fric_vel_u, fric_vel_d);
    }

    void shoot_callback(const shooting_interface::msg::Shoot::SharedPtr msg)
    {
        motor_interface::msg::MotorGoal goal_msg{};

        goal_msg.motor_id.push_back("FEED_R");
        if (msg->feed_state == true) goal_msg.goal_vel.push_back(feed_vel_r);
        else goal_msg.goal_vel.push_back(0);
        goal_msg.goal_pos.push_back(0);

        goal_msg.motor_id.push_back("FRIC_U");
        if (msg->fric_state == true) goal_msg.goal_vel.push_back(fric_vel_u);
        else goal_msg.goal_vel.push_back(0);
        goal_msg.goal_pos.push_back(0);

        goal_msg.motor_id.push_back("FRIC_D");
        if (msg->fric_state == true) goal_msg.goal_vel.push_back(fric_vel_d);
        else goal_msg.goal_vel.push_back(0);
        goal_msg.goal_pos.push_back(0);
        
        motor_pub_->publish(goal_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<ShootLoad>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}