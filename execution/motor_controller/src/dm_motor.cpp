#include "motor_controller/dm_motor.h"
#include <cmath>
#include <linux/can.h>
#include <rclcpp/logger.hpp>
#include "motor_controller/motor_driver.h"
#include "rclcpp/rclcpp.hpp"

umap<int, unique_ptr<CanDriver>> DmMotor::can_drivers; // {port, driver}
umap<int, thread> DmMotor::rx_threads; // {port, thread}
umap<int, can_frame> DmMotor::rx_frames; // {port, frame}
umap<int, umap<int, DmMotor*>> DmMotor::instances; // {port, {hid, instance}}

DmMotor::DmMotor(const string& rid, const int hid, string /*type*/, string port) :
    MotorDriver(rid, hid)
{
    set_mode();
    this->port = port.back() - '0';
    instances[this->port][this->hid] = this; // fb_id = hid

    create_port(this->port);

    turn_on();
    tx_thread = std::thread(&DmMotor::tx_loop, this);
    set_goal(NaN, NaN, 0);
    // kp and kd are not configured yet
}

DmMotor::~DmMotor()
{
    turn_off();
    if (tx_thread.joinable()) tx_thread.join();
    destroy_port(port);
}

void DmMotor::set_param(double p2v_kp, double /*p2v_ki*/, double p2v_kd,
    double /*v2t_kp*/, double /*v2t_ki*/, double /*v2t_kd*/)
{
    kp = p2v_kp;
    kd = p2v_kd;
}

void DmMotor::set_goal(double goal_pos, double goal_vel, double goal_tor)
{
    if (!isnan(goal_pos))
        set_frame(goal_pos, 0, 0, kp, kd);
    else if (!isnan(goal_vel))
        set_frame(0, goal_vel, 0, 0, kd);
    else if (!isnan(goal_tor))
        set_frame(0, 0, goal_tor, 0, 0);
}

tuple<double, double, double> DmMotor::get_state()
{
    return std::make_tuple(fb_pos, fb_vel, fb_tor);
}

void DmMotor::print_info()
{
    RCLCPP_INFO(rclcpp::get_logger("dm_motor"),
        "DmMotor rid %s hid %d created: kp %f, kd %f",
        rid.c_str(), hid, kp, kd
    );
}

void DmMotor::set_frame(float pos, float vel, float tor, float kp, float kd)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(tor,T_MIN, T_MAX, 12);

    tx_frame.data[0] = pos_tmp >> 8;
    tx_frame.data[1] = pos_tmp & 0xff;
    tx_frame.data[2] = vel_tmp >> 4;
    tx_frame.data[3] = (vel_tmp & 0xf) << 4 | kp_tmp >> 8;
    tx_frame.data[4] = kp_tmp & 0xff;
    tx_frame.data[5] = kd_tmp >> 4;
    tx_frame.data[6] = (kd_tmp & 0xf) << 4 | tor_tmp >> 8;
    tx_frame.data[7] = tor_tmp & 0xff;
}

void DmMotor::tx(can_frame frame) const
{
    can_drivers[port]->send_frame(frame);
}

void DmMotor::tx_loop()
{
    while (rclcpp::ok())
    {
        tx(tx_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(TX_FREQ));
    }
}

void DmMotor::turn_on()
{
    can_frame temp_frame;
    temp_frame.can_id = hid;
    uint8_t start_cmd[8] = START_CMD;
    temp_frame.can_dlc = 0x08;
    memcpy(temp_frame.data, start_cmd, sizeof(start_cmd));
    tx(temp_frame);
}

void DmMotor::turn_off()
{
    can_frame temp_frame;
    temp_frame.can_id = hid;
    uint8_t stop_cmd[8] = STOP_CMD;
    temp_frame.can_dlc = 0x08;
    memcpy(temp_frame.data, stop_cmd, sizeof(stop_cmd));
    tx(temp_frame);
}

void DmMotor::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = hid; // can id is equal to motor id in MIT mode
    for (int i = 0; i < 8; i++)
        tx_frame.data[i] = 0;
}

void DmMotor::create_port(int port)
{
    if (can_drivers.find(port) == can_drivers.end())
    {
        can_drivers[port] = std::make_unique<CanDriver>(port);
        rx_threads[port] = std::thread(&DmMotor::rx_loop, port);
    }
}

void DmMotor::destroy_port(int port)
{
    if (instances[port].empty())
    {
        can_drivers.erase(port);
        if (rx_threads[port].joinable()) rx_threads[port].join();
    }
}

int DmMotor::calc_id(can_frame frame)
{
    return frame.data[0];
}

void DmMotor::rx_loop(int port)
{
    auto& can_driver = can_drivers[port];
    auto& can_frame = rx_frames[port];
    auto& instances = DmMotor::instances[port];

    while (rclcpp::ok())
    {
        can_driver->get_frame(can_frame);
        if (can_frame.can_id != 0) continue; // master id must be 0
        int id = calc_id(can_frame);

        // find corresponding instance and let it process
        if (instances.find(id) != instances.end())
            instances[id]->process_rx();
    }
}

void DmMotor::process_rx()
{
    auto& rx_frame = rx_frames[port];

    // get raw data from the frame
    float pos_raw = (rx_frame.data[1]<<8) | (rx_frame.data[2]);
    float vel_raw = (rx_frame.data[3]<<4) | (rx_frame.data[4]>>4);
    float tor_raw = (rx_frame.data[4] & 0xf) << 8 | rx_frame.data[5];

    // write the data to the present_data
    fb_pos = uint_to_float(pos_raw, -P_MAX, P_MAX, 16);
    fb_vel = uint_to_float(vel_raw, -V_MAX, V_MAX, 12);
    fb_tor = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
}

float DmMotor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int DmMotor::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
