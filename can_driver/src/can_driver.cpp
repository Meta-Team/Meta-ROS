#include "can_driver.h"
#include <can_interfaces/msg/detail/mit_data__struct.hpp>
#include <can_interfaces/msg/detail/pos_vel_data__struct.hpp>
#include <can_interfaces/msg/detail/vel_data__struct.hpp>

CanDriver::CanDriver() : Node("can_driver")
{
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // open the CAN socke

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr); // set the interface name

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex; // set the socket address

    bind(s, (struct sockaddr *)&addr, sizeof(addr)); // bind the socket to the CAN interface

    subscription_mit_ = this->create_subscription<can_interfaces::msg::MITData>(
        "mit_data", 10, [this](const can_interfaces::msg::MITData::SharedPtr msg) {
            CanData::mit_interpreter(frame, *msg);
            send_frame();
        });

    subscription_posvel_ = this->create_subscription<can_interfaces::msg::PosVelData>(
        "posvel_data", 10, [this](const can_interfaces::msg::PosVelData::SharedPtr msg) {
            CanData::posvel_interpreter(frame, *msg);
            send_frame();
        });

    subscription_vel_ = this->create_subscription<can_interfaces::msg::VelData>(
        "vel_data", 10, [this](const can_interfaces::msg::VelData::SharedPtr msg)
        {
            CanData::vel_interpreter(frame, *msg);
            send_frame();
        });

    publisher_feedback_ = this->create_publisher<can_interfaces::msg::Feedback>("feedback", 10);
}

CanDriver::~CanDriver()
{
    // close the CAN socket
    close(s);
}

void CanDriver::send_frame()
{
    write(s, &frame, sizeof(frame));
}

// void CanDriver::set_frame_data(mode motor_mode)
// {
//     // set the CAN frame data
//     switch (motor_mode) {
//         case CanDriver::MIT:
//             void mit_interpreter(can_frame &frame, can_interfaces::msg::MITData &data);
//             break;
//         case CanDriver::POSVEL:
//             void posvel_interpreter(can_frame &frame, can_interfaces::msg::PosVelData &data);
//             break;
//         case CanDriver::VEL:
//             void vel_interpreter(can_frame &frame, can_interfaces::msg::VelData &data);
//             break;
//     }
// }

int main(int argc, char *argv[])
{
    // CanDriver can_driver;
    // CanDriver::mode motor_mode = CanDriver::mode::MIT;
    // can_driver.set_frame_data(motor_mode);
    // can_driver.send_frame();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanDriver>());
    rclcpp::shutdown();

    return 0;
}