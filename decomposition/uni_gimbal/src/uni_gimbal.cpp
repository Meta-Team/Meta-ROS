#include "rclcpp/rclcpp.hpp"
#include "uni_gimbal/gimbal.h"
#include <memory>
#include <movement_interface/msg/detail/chassis_move__struct.hpp>
#include <vector>

#include "aiming_interface/msg/uni_aiming.hpp"
#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "motor_interface/msg/motor_goal.hpp"


class UniGimbal: public rclcpp::Node
{
public:
    UniGimbal() : Node("UniGimbal")
    {
        north_offset = this->declare_parameter("north_offset", north_offset);
        init_gimbal();

        goal_sub_ = this->create_subscription<aiming_interface::msg::UniAiming>(
            "uni_aiming", 10, [this](const aiming_interface::msg::UniAiming::SharedPtr msg){
                goal_callback(msg);
            });
        cha_sub_ = this->create_subscription<movement_interface::msg::ChassisMove>(
            "chassis_move", 10, [this](const movement_interface::msg::ChassisMove::SharedPtr msg){
                omega_callback(msg);
            });
        abs_sub_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
            "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
                omega_callback(msg);
            });
        feedback_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "euler_angles", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
                feedback_callback(msg);
            });
        pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_R), [this](){
                pub_callback();
            });
    }

private:
    rclcpp::Subscription<aiming_interface::msg::UniAiming>::SharedPtr goal_sub_;
    rclcpp::Subscription<movement_interface::msg::ChassisMove>::SharedPtr cha_sub_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr feedback_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<Gimbal> gimbal_;

    float north_offset = 0.0; // The offset of the north direction, in radians.

    void goal_callback(const aiming_interface::msg::UniAiming::SharedPtr goal_msg)
    {
        // update goal_dir
        float goal_dir = goal_msg->yaw_pos - north_offset;
        float goal_pitch = goal_msg->pitch_pos;
        gimbal_->set_goal(goal_dir, goal_pitch);
    }

    void feedback_callback(const geometry_msgs::msg::Vector3::SharedPtr feedback_msg)
    {
        float current_dir = - feedback_msg->z;
        float current_pitch = feedback_msg->y;
        gimbal_->get_feedback(current_dir, current_pitch);
    }

    void omega_callback(const movement_interface::msg::ChassisMove::SharedPtr msg)
    {
        gimbal_->update_omega(msg->omega);
    }

    void omega_callback(const movement_interface::msg::AbsoluteMove::SharedPtr msg)
    {
        gimbal_->update_omega(msg->omega);
    }

    void pub_callback()
    {
        auto [yaw_vel, pitch_vel] = gimbal_->calc_vel();
        auto msg = motor_interface::msg::MotorGoal();
        msg.motor_id.push_back("YAW");
        msg.goal_pos.push_back(0.0);
        msg.goal_vel.push_back(yaw_vel);
        msg.motor_id.push_back("PITCH");
        msg.goal_pos.push_back(0.0);
        msg.goal_vel.push_back(pitch_vel);
    }

    void init_gimbal()
    {
        int motor_count = this->declare_parameter("motor_count", 0);

        std::vector<std::string> rids{};
        rids = this->declare_parameter("rids", rids);
        std::vector<double> p2v_kps{};
        p2v_kps = this->declare_parameter("p2v_kps", p2v_kps);
        std::vector<double> p2v_kis{};
        p2v_kis = this->declare_parameter("p2v_kis", p2v_kis);
        std::vector<double> p2v_kds{};
        p2v_kds = this->declare_parameter("p2v_kds", p2v_kds);

        PidParam yaw, pitch;
        bool yaw_found = false, pitch_found = false;

        for (int i = 0; i < motor_count; i++)
        {
            if (rids[i] == "PITCH")
            {
                pitch = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
                pitch_found = true;
            }
            if (rids[i] == "YAW")
            {
                yaw = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
                yaw_found = true;
            }
        }

        if (!yaw_found) RCLCPP_WARN(this->get_logger(), "No yaw motor found in config.");
        if (!pitch_found) RCLCPP_WARN(this->get_logger(), "No pitch motor found in config.");

        gimbal_ = std::make_unique<Gimbal>(yaw, pitch);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UniGimbal>());
    rclcpp::shutdown();
    return 0;
}