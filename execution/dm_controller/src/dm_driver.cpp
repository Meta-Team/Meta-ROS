#include "dm_controller/dm_driver.h"
#include <chrono>
#include <cstdio>
#include <string>

/***********************************************
 * DmDriver is the base class for all drivers.
 ***********************************************/

umap<int, std::unique_ptr<CanDriver>> DmDriver::can_drivers;
umap<int, can_frame> DmDriver::rx_frames;
umap<int, std::thread> DmDriver::rx_threads;
std::vector<std::shared_ptr<DmDriver>> DmDriver::instances;

float DmDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int DmDriver::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

DmDriver::DmDriver(std::string port)
{
    instances.push_back(std::shared_ptr<DmDriver>(this));
    set_port(port.back() - '0');
    last_command = 0.0;
    timeout_thread = std::thread(&DmDriver::check_timeout, this);
}

void DmDriver::check_timeout()
{
    rclcpp::sleep_for(std::chrono::seconds(1));
    while (rclcpp::ok())
    {
        if (rclcpp::Clock().now().seconds() - last_command > 1.0)
        {
            turn_off();
            RCLCPP_INFO(rclcpp::get_logger("dm_driver"), "Motor %s timeout, stop the motor", rid.c_str());
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

void DmDriver::turn_on()
{
    auto frame_temp = tx_frame;

    uint8_t start_cmd[8] = START_CMD;
    tx_frame.can_dlc = 0x08;
    memcpy(tx_frame.data, start_cmd, sizeof(start_cmd));
    tx();
    tx_frame = frame_temp;
}

void DmDriver::turn_off()
{
    uint8_t dlc_temp = tx_frame.can_dlc;

    uint8_t stop_cmd[8] = STOP_CMD;
    tx_frame.can_dlc = 0x08;
    memcpy(tx_frame.data, stop_cmd, sizeof(stop_cmd));
    tx();
    tx_frame.can_dlc = dlc_temp;
}

DmDriver::~DmDriver()
{
    turn_off();
    if (timeout_thread.joinable()) timeout_thread.join();
}

void DmDriver::tx() const
{
    can_drivers[port]->send_frame(tx_frame);
}

void DmDriver::process_rx()
{
    auto& rx_frame = rx_frames[port];

    // get raw data from the frame
    float pos_raw = rx_frame.data[1]<<8 | rx_frame.data[2];
    float vel_raw = rx_frame.data[3]<<4 | rx_frame.data[4]>>4;
    float tor_raw = rx_frame.data[5];

    // write the data to the present_data
    position = uint_to_float(pos_raw, -P_MAX, P_MAX, 16);
    velocity = uint_to_float(vel_raw, -V_MAX, V_MAX, 12);
    torque = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
}

std::tuple<float, float, float> DmDriver::get_state() const
{
    return std::make_tuple(position, velocity, torque);
}

void DmDriver::set_port(int port)
{
    this->port = port;
    if (can_drivers.find(port) == can_drivers.end())
    {
        can_drivers[port] = std::make_unique<CanDriver>(port);
        rx_threads[port] = std::thread(&DmDriver::rx_loop, this, port);
    }
}

void DmDriver::rx_loop(int port)
{
    auto& can_driver = can_drivers[port];
    auto& can_frame = rx_frames[port];
    while (rclcpp::ok())
    {
        can_driver->get_frame(can_frame);
        for (auto& instance : instances)
            instance->process_rx();
    }
}

/*************************************************************************************
 * DmMitDriver is a subclass of DmDriver that represents a specific type of driver.
 ************************************************************************************/

DmMitDriver::DmMitDriver(const std::string& rid, int hid, float kp, float kd, std::string port)
    : DmDriver(port)
{
    printf("DmMitDriver created\n");
    this->rid = rid;
    this->hid = hid;
    set_mode();
    set_param_mit(kp, kd);
}

void DmMitDriver::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = hid;
}

void DmMitDriver::set_param_mit(float kp, float kd)
{
    float tff = 1;
    uint32_t uint_kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t uint_kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t uint_tff = float_to_uint(tff, -T_MAX, T_MAX, 12);
    tx_frame.data[3] &= 0xf0;
    tx_frame.data[3] |= (uint_kp >> 8) & 0x0f;
    tx_frame.data[4] = uint_kp & 0x0ff;
    tx_frame.data[5] = uint_kd >> 4;
    tx_frame.data[6] &= 0x0f;
    tx_frame.data[6] |= (uint_kd & 0x0f) << 4;
    tx_frame.data[6] |= (uint_tff >> 8) & 0x0f;
    tx_frame.data[7] = uint_tff & 0x0ff;
    set_velocity(1);
}

void DmMitDriver::set_velocity(float goal_vel)
{
    last_command = rclcpp::Clock().now().seconds();
    uint32_t uint_vel = float_to_uint(goal_vel, -V_MAX, V_MAX, 12);
    tx_frame.data[3] &= 0x0f;
    tx_frame.data[3] |= (uint_vel & 0x0f) << 4;
    tx_frame.data[2] = uint_vel >> 4;
    tx();
}

void DmMitDriver::set_position(float goal_pos)
{
    last_command = rclcpp::Clock().now().seconds();
    uint32_t uint_pos = float_to_uint(goal_pos, -P_MAX, P_MAX, 16);
    tx_frame.data[1] = uint_pos & 0x0ff;
    tx_frame.data[0] = uint_pos >> 8;
    tx();
}

/*************************************************************************************
 * DmVelDriver is a subclass of DmDriver that represents a specific type of driver.
 ************************************************************************************/

DmVelDriver::DmVelDriver(const std::string& rid, int hid, std::string port)
    : DmDriver(port)
{
    this->hid = hid;
    this->rid = rid;
    set_mode();
}

void DmVelDriver::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = hid;
}

void DmVelDriver::set_velocity(float goal_vel)
{
    last_command = rclcpp::Clock().now().seconds();
    float temp_vel = goal_vel;
    uint32_t* pvel;
    pvel = (uint32_t*) &temp_vel;
    memcpy(&tx_frame.data[0], pvel, sizeof(uint32_t));
    tx();
}

void DmVelDriver::set_position(float /*goal_pos*/)
{
    last_command = rclcpp::Clock().now().seconds();
    // set_position invalid for velocity mode, do nothing
    return;
}