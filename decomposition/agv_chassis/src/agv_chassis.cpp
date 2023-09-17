#include "rclcpp/rclcpp.hpp"

#include "agv_chassis/agv_kinematics.hpp"

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"

class AgvChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<movement_interface::msg::NaturalMove>::SharedPtr nat_sub_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;

    void nat_callback(const movement_interface::msg::NaturalMove::SharedPtr nat_msg)
    {
        motor_pub_->publish(AgvKinematics::natural_decompo(nat_msg));
    }

    void abs_callback(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg)
    {
        motor_pub_->publish(AgvKinematics::absolute_decompo(abs_msg));
    }
public:
    AgvChassis() : Node("AgvChassis")
    {
        nat_sub_ = this->create_subscription<movement_interface::msg::NaturalMove>(
            "natural_move", 10, [this](const movement_interface::msg::NaturalMove::SharedPtr msg){
                this->nat_callback(msg);
            });
        abs_sub_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
            "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
                this->abs_callback(msg);
            });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgvChassis>());
    rclcpp::shutdown();
    return 0;
}