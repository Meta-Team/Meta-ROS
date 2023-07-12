#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

#include "can_interfaces/msg/mit_data.hpp"
#include "can_interfaces/msg/pos_vel_data.hpp"
#include "can_interfaces/msg/vel_data.hpp"
#include "can_interfaces/msg/feedback.hpp"

class CanData
{
public:

    static void mit_interpreter(can_frame &frame, can_interfaces::msg::MITData &data);

    static void posvel_interpreter(can_frame &frame, can_interfaces::msg::PosVelData &data);

    static void vel_interpreter(can_frame &frame, can_interfaces::msg::VelData &data);

    can_interfaces::msg::Feedback feedback_interpreter(const can_frame &frame_read);
};