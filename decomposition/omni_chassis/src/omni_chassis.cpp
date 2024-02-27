#include "rclcpp/rclcpp.hpp"

#include "omni_chassis/omni_kinematics.hpp"
#include "motor_interface/srv/motor_state.hpp"

enum ChassisMode
{
    ALL = 0,
    CHASSIS = 1,
    ABSOLUTE = 2,
    NATURAL = 3,
};

class OmniChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;
    rclcpp::Subscription<movement_interface::msg::ChassisMove>::SharedPtr cha_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Client<gyro_interface::srv::GimbalPosition>::SharedPtr gimbal_cli_;
    rclcpp::Client<motor_interface::srv::MotorState>::SharedPtr motor_cli_;
    rclcpp::CallbackGroup::SharedPtr gimbal_cbgp_;
    rclcpp::CallbackGroup::SharedPtr motor_cbgp_;

    void abs_callback(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg)
    {
        // get gimbal yaw
        float gimbal_yaw = 0;
        auto gimbal_cb = [&](rclcpp::Client<gyro_interface::srv::GimbalPosition>::SharedFuture future){
            gimbal_yaw = future.get()->yaw;
        };
        auto gimbal_req_ = std::make_shared<gyro_interface::srv::GimbalPosition::Request>();
        gimbal_cli_->async_send_request(gimbal_req_, gimbal_cb);

        // get motor position
        float motor_pos = 0;
        auto motor_cb = [&](rclcpp::Client<motor_interface::srv::MotorState>::SharedFuture future){
            motor_pos = future.get()->present_pos[0];
        };
        auto motor_req_ = std::make_shared<motor_interface::srv::MotorState::Request>();
        motor_req_->motor_id.clear();
        motor_req_->motor_id.push_back("YAW");
        motor_cli_->async_send_request(motor_req_, motor_cb);

        // calculate and publish
        auto tx_msg = OmniKinematics::absolute_decompo(abs_msg, gimbal_yaw + motor_pos);
        motor_pub_->publish(tx_msg);
    }

    void cha_callback(const movement_interface::msg::ChassisMove::SharedPtr cha_msg)
    {
        // calculate and publish
        auto tx_msg = OmniKinematics::chassis_decompo(cha_msg);
        motor_pub_->publish(tx_msg);
    }

public:
    OmniChassis() : Node("OmniChassis")
    {
        int chassis_mode = 1;
        chassis_mode = this->declare_parameter("chassis.mode", chassis_mode);
        ChassisMode chassis_mode_ = static_cast<ChassisMode>(chassis_mode);
        RCLCPP_INFO(this->get_logger(), "chassis mode: %d", chassis_mode_);

        // initialize subscriber
        if (chassis_mode_ == ALL || chassis_mode_ == CHASSIS)
        {
            RCLCPP_INFO(this->get_logger(), "chassis movement enabled");
            cha_sub_ = this->create_subscription<movement_interface::msg::ChassisMove>(
                "chassis_move", 10, [this](const movement_interface::msg::ChassisMove::SharedPtr msg){
                    this->cha_callback(msg);
                });
        }
        if (chassis_mode_ == ALL || chassis_mode_ == ABSOLUTE)
        {
            abs_sub_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
                "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
                    this->abs_callback(msg);
                });
            RCLCPP_INFO(this->get_logger(), "absolute movement enabled");

            // initialize client
            gimbal_cbgp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            motor_cbgp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            gimbal_cli_ = this->create_client<gyro_interface::srv::GimbalPosition>("gimbal_position", rmw_qos_profile_services_default, gimbal_cbgp_);
            motor_cli_ = this->create_client<motor_interface::srv::MotorState>("motor_present", rmw_qos_profile_services_default, motor_cbgp_);
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->get_node_base_interface());
            
            // wait for services
            abs_wait();
        }

        // initialize publisher
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);

        RCLCPP_INFO(this->get_logger(), "OmniChassis initialized");
    }

    void abs_wait()
    {
        while (!gimbal_cli_->wait_for_service(std::chrono::seconds(1)) ||
               !motor_cli_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                // rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        RCLCPP_INFO(this->get_logger(), "service available now");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto omni_chassis = std::make_shared<OmniChassis>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(omni_chassis->get_node_base_interface());
    executor.spin();
    return 0;
}