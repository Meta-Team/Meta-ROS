#include "rclcpp/rclcpp.hpp"

#include "shooting_interface/msg/shoot.hpp"
#include "motor_interface/msg/motor_goal.hpp"

#define FEED_VEL 300
#define FRIC_VEL 300

enum MotorId
{
    FRIC1 = 5,
    FRIC2 = 6,
    FEED1 = 9,
    FEED2 = 10,
};

class ShootLoad : public rclcpp::Node
{
public:
    ShootLoad() : Node("shoot_load")
    {
        shoot_sub_ = this->create_subscription<shooting_interface::msg::Shoot>("shoot",
            10, std::bind(&ShootLoad::shoot_callback, this, std::placeholders::_1));
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
    }

private:
    rclcpp::Subscription<shooting_interface::msg::Shoot>::SharedPtr shoot_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;

    void shoot_callback(const shooting_interface::msg::Shoot::SharedPtr msg)
    {
        motor_interface::msg::MotorGoal goal_msg{};
        if (msg->id == 0)
        {
            goal_msg.motor_id.push_back(FEED1);
            if (msg->feed_state == true) goal_msg.goal_vel.push_back(FEED_VEL);
            else goal_msg.goal_vel.push_back(0);
            goal_msg.goal_pos.push_back(0);

            goal_msg.motor_id.push_back(FRIC1);
            if (msg->fric_state == true) goal_msg.goal_vel.push_back(FRIC_VEL);
            else goal_msg.goal_vel.push_back(0);
            goal_msg.goal_pos.push_back(0);
            motor_pub_->publish(goal_msg);
        }
        else if (msg->id == 1)
        {
            goal_msg.motor_id.push_back(FEED2);
            if (msg->feed_state == true) goal_msg.goal_vel.push_back(FEED_VEL);
            else goal_msg.goal_vel.push_back(0);
            goal_msg.goal_pos.push_back(0);

            goal_msg.motor_id.push_back(FRIC2);
            if (msg->fric_state == true) goal_msg.goal_vel.push_back(FRIC_VEL);
            else goal_msg.goal_vel.push_back(0);
            goal_msg.goal_pos.push_back(0);
            motor_pub_->publish(goal_msg);
        }
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