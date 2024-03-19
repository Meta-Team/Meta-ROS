#include "rclcpp/rclcpp.hpp"

#include "behavior_interface/msg/shoot.hpp"
#include "motor_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

class ShootLoad : public rclcpp::Node
{
public:
    ShootLoad() : Node("shoot_load")
    {
        shoot_sub_ = this->create_subscription<behavior_interface::msg::Shoot>("shoot",
            10, std::bind(&ShootLoad::shoot_callback, this, std::placeholders::_1));
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        vib_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&ShootLoad::vib_timer_callback, this));
        get_param();
        RCLCPP_INFO(this->get_logger(), "ShootLoad initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Shoot>::SharedPtr shoot_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr vib_timer_;

    float feed_vel_l = 5.0, feed_vel_r = 5.0;
    float fric_vel_u = 300.0, fric_vel_d = 300.0;
    float feed_ratio = 36.0, fric_ratio = 1.0;
    float feed_vib_amp = 1.0, feed_vib_freq = 1.0;
    float feed_vib = 0.0;

    void get_param()
    {
        feed_vel_l = this->declare_parameter("gimbal.feed_vel_l", feed_vel_l);
        feed_vel_r = this->declare_parameter("gimbal.feed_vel_r", feed_vel_r);
        fric_vel_u = this->declare_parameter("gimbal.fric_vel_u", fric_vel_u);
        fric_vel_d = this->declare_parameter("gimbal.fric_vel_d", fric_vel_d);
        feed_ratio = this->declare_parameter("gimbal.feed_ratio", feed_ratio);
        fric_ratio = this->declare_parameter("gimbal.fric_ratio", fric_ratio);
        RCLCPP_INFO(this->get_logger(), "Feed velocity: %f and %f, Friction velocity: %f and %f", feed_vel_l, feed_vel_r, fric_vel_u, fric_vel_d);
        feed_vib_freq = this->declare_parameter("gimbal.feed_vib_freq", feed_vib_freq);
        feed_vib_amp = this->declare_parameter("gimbal.feed_vib_amp", feed_vib_amp);
        RCLCPP_INFO(this->get_logger(), "Feed vibration frequency: %f and amplitude: %f", feed_vib_freq, feed_vib_amp);
    }

    void shoot_callback(const behavior_interface::msg::Shoot::SharedPtr msg)
    {
        motor_interface::msg::MotorGoal goal_msg{};

        goal_msg.motor_id.push_back("FEED_L");
        if (msg->feed_state == true) goal_msg.goal_vel.push_back((feed_vel_l + feed_vib) * feed_ratio);
        else goal_msg.goal_vel.push_back(feed_vib * feed_ratio);
        goal_msg.goal_pos.push_back(NaN);

        goal_msg.motor_id.push_back("FEED_R");
        if (msg->feed_state == true) goal_msg.goal_vel.push_back((feed_vel_r + feed_vib) * feed_ratio);
        else goal_msg.goal_vel.push_back(feed_vib * feed_ratio);
        goal_msg.goal_pos.push_back(NaN);

        goal_msg.motor_id.push_back("FRIC_U");
        if (msg->fric_state == true) goal_msg.goal_vel.push_back(fric_vel_u * fric_ratio);
        else goal_msg.goal_vel.push_back(0);
        goal_msg.goal_pos.push_back(NaN);

        goal_msg.motor_id.push_back("FRIC_D");
        if (msg->fric_state == true) goal_msg.goal_vel.push_back(fric_vel_d * fric_ratio);
        else goal_msg.goal_vel.push_back(0);
        goal_msg.goal_pos.push_back(NaN);
        
        motor_pub_->publish(goal_msg);
    }

    void vib_timer_callback()
    {
        feed_vib = feed_vib_amp * sin(2 * M_PI * feed_vib_freq * rclcpp::Clock().now().seconds());
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