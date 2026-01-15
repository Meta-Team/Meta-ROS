#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <iostream>

// Set this to match your SRDF group name
static const std::string PLANNING_GROUP = "meta_engineer_arm";

int main(int argc, char** argv)
{
    // 1. Initialize ROS
    rclcpp::init(argc, argv);
    
    // 2. Create the Node
    // We use NodeOptions with "automatically_declare_parameters_from_overrides" 
    // to easily ingest the robot_description params passed by the launch file.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("engineer26_planner", node_options);

    // 3. Spin ROS in a background thread
    // MoveIt needs this spinner to process TF and JointState updates continuously.
    // If we didn't do this, the robot would never know where it is.
    std::thread spinner_thread([node]() {
        rclcpp::spin(node);
    });

    // 4. Create MoveGroupInterface
    // It is safe to create this now because the node is fully initialized.
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

    // Basic Config
    move_group->setMaxVelocityScalingFactor(0.5);
    move_group->setMaxAccelerationScalingFactor(0.5);
    move_group->setPlanningTime(5.0);

    // 5. Main Control Loop (CLI)
    std::cout << "=========================================" << std::endl;
    std::cout << "   Engineer26 Simple Planner Ready" << std::endl;
    std::cout << "   Planning Group: " << PLANNING_GROUP << std::endl;
    std::cout << "=========================================" << std::endl;

    while (rclcpp::ok()) {
        double x, y, z;
        std::cout << "\nEnter Target X Y Z (space separated, or 'q' to quit): ";
        
        // Peek to check for 'q'
        std::cin >> std::ws; 
        char check_quit = std::cin.peek();

        if (check_quit == 'q' || check_quit == 'Q') {
            break;
        }

        if (!(std::cin >> x >> y >> z)) {
            std::cout << "Invalid input. Try again." << std::endl;
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            continue;
        }

        // Get current pose to preserve orientation
        geometry_msgs::msg::Pose target_pose = move_group->getCurrentPose().pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        std::cout << "Planning to [" << x << ", " << y << ", " << z << "]..." << std::endl;

        move_group->setPoseTarget(target_pose);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            std::cout << "Plan FOUND! Execute? (y/n): ";
            char confirm;
            std::cin >> confirm;
            if (confirm == 'y' || confirm == 'Y') {
                std::cout << "Executing..." << std::endl;
                move_group->execute(my_plan); // This BLOCKS until movement finishes
                std::cout << "Movement Complete." << std::endl;
            } else {
                std::cout << "Cancelled." << std::endl;
            }
        } else {
            std::cout << "Plan FAILED. Check collision or workspace limits." << std::endl;
        }
    }

    // 6. Cleanup
    rclcpp::shutdown();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }

    return 0;
}