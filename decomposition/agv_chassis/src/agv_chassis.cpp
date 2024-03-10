#include "rclcpp/rclcpp.hpp"
#include "agv_chassis/agv_kinematics.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "motor_interface/msg/motor_state.hpp"
#include <cstdint>
#include <vector>

namespace ph = std::placeholders;
using std::string, std::vector;

class AgvChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<motor_interface::msg::MotorState>::SharedPtr motor_sub_;

    void nat_callback(const behavior_interface::msg::Move::SharedPtr nat_msg)
    {
        // MY_TODO: complete the algorithm
        float temp = 0;

        // calculate and publish
        motor_pub_->publish(AgvKinematics::natural_decompo(nat_msg, temp));
    }

    void abs_callback(const behavior_interface::msg::Move::SharedPtr abs_msg)
    {
        // MY_TODO: complete the algorithm
        float temp = 0;
        
        // calculate and publish
        motor_pub_->publish(AgvKinematics::absolute_decompo(abs_msg, temp));
    }

    void cha_callback(const behavior_interface::msg::Move::SharedPtr cha_msg)
    {
        motor_pub_->publish(AgvKinematics::chassis_decompo(cha_msg));
    }

    void gimbal_callback(const geometry_msgs::msg::Vector3::SharedPtr gimbal_msg)
    {
        // MY_TODO: complete the algorithm
    }
    
    void motor_callback(const motor_interface::msg::MotorState::SharedPtr motor_msg)
    {
        // MY_TODO: complete the algorithm
    }

public:
    AgvChassis() : Node("AgvChassis")
    {
        string mode = "chassis";
        mode = this->declare_parameter("chassis.move_mode", mode);
        RCLCPP_INFO(this->get_logger(), "Chassis mode: %s", mode.c_str());

        get_offsets();

        // initialize subscriber
        gimbal_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("euler_angles", 10,
            std::bind(&AgvChassis::gimbal_callback, this, ph::_1));
        motor_sub_ = this->create_subscription<motor_interface::msg::MotorState>("motor_state", 10,
            std::bind(&AgvChassis::motor_callback, this, ph::_1));
        
        if (mode == "chassis")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::cha_callback, this, ph::_1));
        else if (mode == "natural")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::nat_callback, this, ph::_1));
        else if (mode == "absolute")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::abs_callback, this, ph::_1));
        else
            RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s", mode.c_str());
        // MY_TODO: other subscriber

        // initialize publisher
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        RCLCPP_INFO(this->get_logger(), "AgvChassis initialized");
    }

    void get_offsets()
    {
        vector<double> offsets;
        offsets = this->declare_parameter("chassis.offsets", offsets);
        vector<string> rids;
        rids = this->declare_parameter("chassis.rids", rids);

        for (int i = 0; i < static_cast<int>(rids.size()); i++)
        {
            const string id = rids[i];
            const float offset = offsets[i];
            const auto it = AgvKinematics::offsets.find(id);
            if (it != AgvKinematics::offsets.end()) // found
            {
                // {rid, {offset, found}}
                auto& [offset_, found_] = it->second;
                offset_ = offset;
                found_ = true;
            }
        }

        for (const auto& [rid, offset_found] : AgvKinematics::offsets)
        {
            const auto& [offset, found] = offset_found;
            if (!found) RCLCPP_ERROR(this->get_logger(), "Offset not found for %s", rid.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AgvChassis>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}