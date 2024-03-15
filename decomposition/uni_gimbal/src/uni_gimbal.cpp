#include "rclcpp/rclcpp.hpp"
#include "uni_gimbal/gimbal.h"
#include <memory>
#include <vector>

#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "motor_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

class UniGimbal: public rclcpp::Node
{
public:
    UniGimbal() : Node("UniGimbal")
    {
        north_offset = this->declare_parameter("north_offset", north_offset);
        init_gimbal();

        goal_sub_ = this->create_subscription<behavior_interface::msg::Aim>(
            "aim", 10, [this](const behavior_interface::msg::Aim::SharedPtr msg){
                goal_callback(msg);
            });
        move_sub_ = this->create_subscription<behavior_interface::msg::Move>(
            "move", 10, [this](const behavior_interface::msg::Move::SharedPtr msg){
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

        RCLCPP_INFO(this->get_logger(), "UniGimbal initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Aim>::SharedPtr goal_sub_;
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr feedback_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<Gimbal> gimbal_;

    float north_offset = 0.0; // The offset of the north direction, in radians.

    void goal_callback(const behavior_interface::msg::Aim::SharedPtr goal_msg)
    {
        // update goal_dir
        float goal_dir = goal_msg->yaw; // relative to north
        float goal_pitch = goal_msg->pitch;
        gimbal_->set_goal(goal_dir, goal_pitch);
    }

    void feedback_callback(const geometry_msgs::msg::Vector3::SharedPtr feedback_msg)
    {
        float current_dir = - feedback_msg->z; // relative to north
        float current_pitch = feedback_msg->y;
        gimbal_->get_feedback(current_dir, current_pitch);
    }

    void omega_callback(const behavior_interface::msg::Move::SharedPtr msg)
    {
        gimbal_->update_omega(msg->omega);
    }

    void pub_callback()
    {
        auto [yaw_vel, pitch_vel] = gimbal_->calc_vel();
        auto msg = motor_interface::msg::MotorGoal();
        msg.motor_id.push_back("YAW");
        msg.goal_pos.push_back(NaN);
        msg.goal_vel.push_back(yaw_vel);
        msg.motor_id.push_back("PITCH");
        msg.goal_pos.push_back(NaN);
        msg.goal_vel.push_back(pitch_vel);
        pub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Yaw vel: %f, pitch vel: %f", yaw_vel, pitch_vel);
    }

    void init_gimbal()
    {
        int motor_count = this->declare_parameter("motor.count", 0);

        std::vector<std::string> rids{};
        rids = this->declare_parameter("motor.rids", rids);
        std::vector<double> p2v_kps{};
        p2v_kps = this->declare_parameter("motor.p2v.kps", p2v_kps);
        std::vector<double> p2v_kis{};
        p2v_kis = this->declare_parameter("motor.p2v.kis", p2v_kis);
        std::vector<double> p2v_kds{};
        p2v_kds = this->declare_parameter("motor.p2v.kds", p2v_kds);

        PidParam yaw, pitch;
        bool yaw_found = false, pitch_found = false;

        for (int i = 0; i < motor_count; i++)
        {
            if (rids[i] == "PITCH")
            {
                pitch = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
                pitch_found = true;
                RCLCPP_INFO(this->get_logger(), "Pitch kp: %f, ki: %f, kd: %f", p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            }
            if (rids[i] == "YAW")
            {
                yaw = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
                yaw_found = true;
                RCLCPP_INFO(this->get_logger(), "Yaw kp: %f, ki: %f, kd: %f", p2v_kps[i], p2v_kis[i], p2v_kds[i]);
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