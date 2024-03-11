#include "rclcpp/rclcpp.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "vision_interface/msg/auto_aim.hpp"
#include <behavior_interface/msg/detail/aim__struct.hpp>
#include <rclcpp/timer.hpp>

#define COUNT 10

#define UPDATE_R 10 // ms
#define SEARCH_VEL 0.3 // rad/s // MY_TODO: change to param

using namespace behavior_interface::msg;
using vision_interface::msg::AutoAim;

class AutoSentry : public rclcpp::Node
{
public:
    AutoSentry() : Node("auto_sentry")
    {
        this->aim_pub_ = this->create_publisher<Aim>("aim", 10);
        this->move_pub_ = this->create_publisher<Move>("move", 10);
        this->shoot_pub_ = this->create_publisher<Shoot>("shoot", 10);
    }

private:
    rclcpp::Publisher<Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<Move>::SharedPtr move_pub_;
    rclcpp::Publisher<Shoot>::SharedPtr shoot_pub_;
    rclcpp::Subscription<AutoAim>::SharedPtr auto_aim_sub_;
    rclcpp::TimerBase::SharedPtr count_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    int count_down = COUNT;

    Aim aim_msg;
    Move move_msg;
    Shoot shoot_msg;

    void auto_aim_callback(const AutoAim::SharedPtr msg)
    {
        aim_msg.yaw = msg->yaw;
        aim_msg.pitch = msg->pitch;
        count_down = COUNT; // reset count down
    }

    void count_timer_callback()
    {
        --count_down;
    }

    void update_timer_callback()
    {
        if (count_down <= 0)
        {

        }
    }

    void pub_timer_callback()
    {
    }

    void search()
    {
        aim_msg.yaw += SEARCH_VEL * UPDATE_R / 1000;
        aim_msg.yaw = std::fmod(aim_msg.yaw, 2 * M_PI);

        // aim_msg.pitch
        // start sin wave
        // aim_msg.pitch = std::sin(aim_msg.yaw);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AutoSentry>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}