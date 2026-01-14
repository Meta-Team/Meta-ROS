#include <functional>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "operation_interface/msg/dbus_control.hpp"


class Engineer26 : public rclcpp::Node
{
public:
    Engineer26(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("engineer26", options)
    {
        enable_moveit = true;
        initmove = false;
        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "/dbus_control", 10, std::bind(&Engineer26::dbus_callback, this, std::placeholders::_1));
        goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&Engineer26::clicked_point_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Engineer26 init");
    }

    void init_moveit()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "meta_engineer_arm");

        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);

        RCLCPP_INFO(this->get_logger(), "[test_engineer] init moveit sucess") ;
    }

    bool initmove;

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    bool enable_moveit;

    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        double ls_x, ls_y, rs_x, rs_y;
        std::string lsw = msg->lsw, rsw = msg->rsw;
        bool wheel;
        if (lsw == "MID")
            enable_moveit = true;
        else
            enable_moveit = false;
    }
    void clicked_point_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!enable_moveit)
            return;
        if (!initmove)
            init_moveit(),
            initmove = true;
        RCLCPP_INFO(this->get_logger(), "Received point: (%f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        geometry_msgs::msg::Pose target_pose;
        target_pose.set__position(msg->pose.position);
        target_pose.set__orientation(msg->pose.orientation);
        // orientation

        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(this->get_logger(), "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Success! Executing...");
            move_group_->execute(my_plan);
        }
        else
            RCLCPP_ERROR(this->get_logger(), "Plan Failed! Check workspace limits or collision.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Engineer26>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    node->init_moveit(),
    node->initmove = false;
    executor.spin();
    rclcpp::shutdown();
    return 0;
}