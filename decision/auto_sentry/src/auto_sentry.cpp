#include "rclcpp/rclcpp.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "vision_interface/msg/auto_aim.hpp"
#include "auto_sentry/aim_mode.hpp"

#define UPDATE_R 20 // ms
#define PUB_R 20 // ms

using namespace behavior_interface::msg;
using vision_interface::msg::AutoAim;
 
class AutoSentry : public rclcpp::Node
{
public:
    AutoSentry() : Node("auto_sentry"), aim_mode(this->get_logger())
    {
        this->aim_pub_ = this->create_publisher<Aim>("aim", 10);
        this->move_pub_ = this->create_publisher<Move>("move", 10);
        this->shoot_pub_ = this->create_publisher<Shoot>("shoot", 10);

        search_vel = this->declare_parameter("gimbal.search_vel", search_vel);
        freq = this->declare_parameter("gimbal.pitch_freq", freq);
        amplitude = this->declare_parameter("gimbal.pitch_amp", amplitude);
        north_offset = this->declare_parameter("north_offset", north_offset);
        auto_rotate = this->declare_parameter("auto_rotate", auto_rotate);

        set_msgs();

        this->auto_aim_sub_ = this->create_subscription<AutoAim>("auto_aim", 10,
            std::bind(&AutoSentry::auto_aim_callback, this, std::placeholders::_1));

        this->publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_R),
            std::bind(&AutoSentry::pub_timer_callback, this));
        this->update_timer_ = this->create_wall_timer(std::chrono::milliseconds(UPDATE_R),
            std::bind(&AutoSentry::update_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "AutoSentry initialized");
    }

private:
    rclcpp::Publisher<Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<Move>::SharedPtr move_pub_;
    rclcpp::Publisher<Shoot>::SharedPtr shoot_pub_;
    rclcpp::Subscription<AutoAim>::SharedPtr auto_aim_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    double north_offset = 0.0;
    double search_vel = 0.5; // rad/s
    double freq = 3.0;
    double amplitude = 0.4; // rad
    double auto_rotate = 2; // rad/s

    AimMode aim_mode;

    float yaw_buffer = 0.0;
    float pitch_buffer = 0.0;
    bool received = false;
    Aim aim_msg;
    Move move_msg;
    Shoot shoot_msg;

    void auto_aim_callback(const AutoAim::SharedPtr msg)
    {
        received = true;
        // save the latest aim
        yaw_buffer = msg->yaw;
        pitch_buffer = msg->pitch;
    }

    void update_timer_callback()
    {
        if (received) ++aim_mode;
        else --aim_mode;

        received = false; // reset

        if (aim_mode.is_active()) // target found
        {
            // load the latest aim
            aim_msg.yaw = yaw_buffer;
            aim_msg.pitch = pitch_buffer;
            // start feeding
            shoot_msg.feed_state = true;
        }
        else { // target lost
            search();
            // stop feeding
            shoot_msg.feed_state = false;
        }
    }

    void pub_timer_callback()
    {
        aim_pub_->publish(aim_msg);
        move_pub_->publish(move_msg);
        shoot_pub_->publish(shoot_msg);
    }

    void search()
    {
        // yaw
        aim_msg.yaw += search_vel * UPDATE_R / 1000;
        aim_msg.yaw = std::fmod(aim_msg.yaw, 2 * M_PI);

        // pitch sin wave
        aim_msg.pitch = amplitude * std::sin(freq * now().seconds());
    }

    void set_msgs()
    {
        aim_msg.yaw = north_offset;
        aim_msg.pitch = 0.0;

        move_msg.vel_x = 0.0;
        move_msg.vel_y = 0.0;
        move_msg.omega = auto_rotate;

        shoot_msg.id = 0;
        shoot_msg.feed_state = false;
        shoot_msg.fric_state = true;
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