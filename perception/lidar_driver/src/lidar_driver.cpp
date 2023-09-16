#include "rclcpp/rclcpp.hpp"
#include <lidar_interface/msg/detail/lidar_dist__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <iostream>
#include <map>
#include "termios.h"

#include "lidar_driver/lidar_serial_driver.h"
#include "lidar_interface/msg/lidar_dist.hpp"

class LidarDriver : public rclcpp::Node
{
private:
    rclcpp::Publisher<lidar_interface::msg::LidarDist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        auto msg = lidar_interface::msg::LidarDist();

        LidarSerialDriver lidar_serial_driver(0);
        lidar_serial_driver.receive(msg.dist);
        pub_->publish(msg);
    }

public:
    LidarDriver(): Node("LidarDriver")
    {
        pub_ = this->create_publisher<lidar_interface::msg::LidarDist>("lidar_dist", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&LidarDriver::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarDriver>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}