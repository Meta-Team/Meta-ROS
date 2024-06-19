#include "rclcpp/rclcpp.hpp"
#include "uni_gimbal/gimbal.h"
#include <memory>
#include <vector>

#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "device_interface/msg/motor_goal.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define NaN std::nan("")

#define CONTROL_R 4 // ms

class UniGimbal: public rclcpp::Node
{
public:
    UniGimbal() : Node("UniGimbal")
    {
        init_gimbal();

        goal_sub_ = this->create_subscription<behavior_interface::msg::Aim>(
            "aim", 10, [this](const behavior_interface::msg::Aim::SharedPtr msg){
                goal_callback(msg);
            });
        angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "euler_angles", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg){
                angle_callback(msg);
            });
#if IMU_FB == true
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg){
                imu_callback(msg);
            });
#endif // IMU_FB
        pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_R), [this](){
                pub_callback();
            });

        RCLCPP_INFO(this->get_logger(), "UniGimbal initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Aim>::SharedPtr goal_sub_;
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_sub_;
#if IMU_FB == true
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#endif // IMU_FB
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<Gimbal> gimbal_;

    double yaw_offset = 0.0;
    double pitch_offset = 0.0;

    void goal_callback(const behavior_interface::msg::Aim::SharedPtr goal_msg)
    {
        // update goal_dir
        double goal_dir = goal_msg->yaw; // relative to north
        double goal_pitch = goal_msg->pitch;
        gimbal_->set_goal(goal_dir, goal_pitch);
    }

    void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr angle_msg)
    {
        double current_dir = - angle_msg->z + yaw_offset; // relative to north
        double current_pitch = angle_msg->y + pitch_offset;
        gimbal_->update_pos_feedback(current_dir, current_pitch);
    }

#if IMU_FB == true
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        double yaw_vel = imu_msg->angular_velocity.z;
        double pitch_vel = - imu_msg->angular_velocity.y;
        gimbal_->update_vel_feedback(yaw_vel, pitch_vel);
    }
#endif // IMU_FB

    void pub_callback()
    {
        auto msg = gimbal_->get_motor_goal();
        pub_->publish(msg);
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
#if IMU_FB == true
        std::vector<double> v2c_kps{};
        v2c_kps = this->declare_parameter("motor.v2c.kps", v2c_kps);
        std::vector<double> v2c_kis{};
        v2c_kis = this->declare_parameter("motor.v2c.kis", v2c_kis);
        std::vector<double> v2c_kds{};
        v2c_kds = this->declare_parameter("motor.v2c.kds", v2c_kds);
#endif // IMU_FB

        yaw_offset = this->declare_parameter("gimbal.aim_yaw_offset", yaw_offset);
        pitch_offset = this->declare_parameter("gimbal.aim_pitch_offset", pitch_offset);

        PidParam yaw_p2v, pitch_p2v;
#if IMU_FB == true
        PidParam yaw_v2v, pitch_v2v;
#endif // IMU_FB
        bool yaw_found = false, pitch_found = false;

        for (int i = 0; i < motor_count; i++)
        {
            if (rids[i] == "PITCH")
            {
                pitch_p2v = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
#if IMU_FB == true
                pitch_v2v = PidParam(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
#endif // IMU_FB
                pitch_found = true;
                RCLCPP_INFO(this->get_logger(), "Pitch kp: %f, ki: %f, kd: %f", p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            }
            if (rids[i] == "YAW")
            {
                yaw_p2v = PidParam(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
#if IMU_FB == true
                yaw_v2v = PidParam(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
#endif // IMU_FB
                yaw_found = true;
                RCLCPP_INFO(this->get_logger(), "Yaw kp: %f, ki: %f, kd: %f", p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            }
        }

        if (!yaw_found) RCLCPP_WARN(this->get_logger(), "No yaw motor found in config.");
        if (!pitch_found) RCLCPP_WARN(this->get_logger(), "No pitch motor found in config.");

#if IMU_FB == false
        gimbal_ = std::make_unique<Gimbal>(yaw_p2v, pitch_p2v);
#else
        gimbal_ = std::make_unique<Gimbal>(yaw_p2v, pitch_p2v, yaw_v2v, pitch_v2v);
#endif // IMU_FB
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UniGimbal>());
    rclcpp::shutdown();
    return 0;
}