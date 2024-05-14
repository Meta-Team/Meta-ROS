#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/key_mouse.hpp"
#include "operation_interface/msg/custom_controller.hpp"
#include "behavior_interface/msg/end_vel.hpp"

#define PUB_RATE 10

class CcArm : public rclcpp::Node
{
public:
    CcArm() : Node("cc_arm")
    {
        cc_sub_ = this->create_subscription<operation_interface::msg::CustomController>(
            "custom_controller", 10, std::bind(&CcArm::cc_callback, this, std::placeholders::_1));
        km_sub_ = this->create_subscription<operation_interface::msg::KeyMouse>(
            "key_mouse", 10, std::bind(&CcArm::km_callback, this, std::placeholders::_1));
        end_vel_pub_ = this->create_publisher<behavior_interface::msg::EndVel>("end_vel", 10);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&CcArm::pub_timer_callback, this));

        RCLCPP_INFO(get_logger(), "CcArm initialized.");
    }
    
private:
    bool enable = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    rclcpp::Subscription<operation_interface::msg::CustomController>::SharedPtr cc_sub_;
    rclcpp::Subscription<operation_interface::msg::KeyMouse>::SharedPtr km_sub_;
    rclcpp::Publisher<behavior_interface::msg::EndVel>::SharedPtr end_vel_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    void km_callback(const operation_interface::msg::KeyMouse::SharedPtr msg)
    {
        if (!msg->active) return;
        enable = msg->left_button;
    }

    void cc_callback(const operation_interface::msg::CustomController::SharedPtr msg)
    {
        x = msg->x_vel;
        y = msg->y_vel;
        z = msg->z_vel;
        yaw = msg->yaw_vel;
        pitch = msg->pitch_vel;
        roll = msg->roll_vel;
    }

    void pub_timer_callback()
    {
        behavior_interface::msg::EndVel end_vel_msg;
        if (enable)
        {
            end_vel_msg.x = x;
            end_vel_msg.y = y;
            end_vel_msg.z = z;
            end_vel_msg.yaw = yaw;
            end_vel_msg.pitch = pitch;
            end_vel_msg.roll = roll;
        }
        else // if not enabled, set all velocities to zero
        {
            end_vel_msg.x = 0.0;
            end_vel_msg.y = 0.0;
            end_vel_msg.z = 0.0;
            end_vel_msg.yaw = 0.0;
            end_vel_msg.pitch = 0.0;
            end_vel_msg.roll = 0.0;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CcArm>());
    rclcpp::shutdown();
    return 0;
}