#include "rclcpp/rclcpp.hpp"

#include "motor_interface/msg/motor_goal.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <string>

#define NaN std::nan("")

class MotorTester : public rclcpp::Node
{
public:
    MotorTester() : Node("motor_tester")
    {
        // get param
        motor_id = this->declare_parameter("motor_rid", "");
        upper_bound = this->declare_parameter("goal_upper", upper_bound);
        lower_bound = this->declare_parameter("goal_lower", lower_bound);
        goal = this->declare_parameter("goal_init", goal);
        goal_sens = this->declare_parameter("sensitivity", goal_sens);
        deadzone = this->declare_parameter("deadzone", deadzone);
        std::string mode_ = this->declare_parameter("goal_type", "");
        if (mode_ == "pos") mode = POS;
        else if (mode_ == "vel") mode = VEL;
        else if (mode_ == "tor") mode = TOR;
        else {
            RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s", mode_.c_str());
            return;
        }

        // pub and sub
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&MotorTester::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MotorTester initialized with motor_id: %s, goal_upper: %f, goal_lower: %f, goal_init: %f, sensitivity: %f, mode: %s",
            motor_id.c_str(), upper_bound, lower_bound, goal, goal_sens, mode_.c_str());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;

    std::string motor_id; // id of motor
    double goal = 0.0; // goal of motor, vel or pos
    double goal_sens = 0.1; // sensitivity of goal change
    double upper_bound = 0.0; // upper bound of goal
    double lower_bound = -0.0; // lower bound of goal
    double deadzone = 0.0; // deadzone of joy
    enum {POS, VEL, TOR} mode; // mode of motor, vel or pos
    double last_receive = 0.0; // interval of goal change
    bool first_receive = true; // first receive of joy

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // use left stick to control goal, forward is positive
        if (first_receive)
        {
            last_receive = rclcpp::Clock().now().seconds();
            first_receive = false;
            return;
        }
        double interval = rclcpp::Clock().now().seconds() - last_receive;
        last_receive = rclcpp::Clock().now().seconds();
        goal += apply_deadzone(msg->axes[1], deadzone) * goal_sens * interval;
        curb(goal, upper_bound, lower_bound);

        motor_interface::msg::MotorGoal motor_goal{};
        motor_goal.motor_id.push_back(motor_id);
        switch (mode)
        {
        case POS:
            motor_goal.goal_pos.push_back(goal);
            motor_goal.goal_vel.push_back(NaN);
            motor_goal.goal_tor.push_back(NaN);
            break;
        case VEL:
            motor_goal.goal_pos.push_back(NaN);
            motor_goal.goal_vel.push_back(goal);
            motor_goal.goal_tor.push_back(NaN);
            break;
        case TOR:
            motor_goal.goal_pos.push_back(NaN);
            motor_goal.goal_vel.push_back(NaN);
            motor_goal.goal_tor.push_back(goal);
            break;
        }
        motor_pub_->publish(motor_goal);
    }

    void curb(double &val, const double& upper, const double& lower)
    {
        if (val > upper) val = upper;
        else if (val < lower) val = lower;
    }

    double apply_deadzone(const double& val, const double& deadzone)
    {
        if (val < deadzone && val > -deadzone) return 0.0;
        else if (val > 0) return (val - deadzone) / (1 - deadzone);
        else return (val + deadzone) / (1 - deadzone);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTester>());
    rclcpp::shutdown();
    return 0;
}