#include "rclcpp/rclcpp.hpp"

#include "device_interface/msg/angular_position.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class AhrsInterpreter : public rclcpp::Node
{
public:
    AhrsInterpreter() : Node("ahrs_interpreter")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "euler_angles", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
                callback(msg);
            });
        pub_ = this->create_publisher<device_interface::msg::AngularPosition>("angular_position", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
    rclcpp::Publisher<device_interface::msg::AngularPosition>::SharedPtr pub_;

    void callback(const geometry_msgs::msg::Vector3::SharedPtr feedback_msg)
    {
        device_interface::msg::AngularPosition pub_msg;
        pub_msg.yaw = 2 * M_PI - feedback_msg->z;
        pub_msg.pitch = feedback_msg->y;
        pub_->publish(pub_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AhrsInterpreter>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}