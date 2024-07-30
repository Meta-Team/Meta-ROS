// data decoding is inspired by https://gitee.com/SMBU-POLARBEAR/MotorDrive

#include "motor_controller/mi_motor.h"
#include "motor_controller/motor_driver.h"
#include <cmath>
#include <cstdint>
#include <linux/can.h>
#include <rclcpp/utilities.hpp>
#include <thread>
#include <tuple>
#include "rclcpp/rclcpp.hpp"

umap<int, unique_ptr<CanDriver>> MiMotor::can_drivers;
umap<int, std::thread> MiMotor::rx_threads;
umap<int, can_frame> MiMotor::rx_frames;
umap<int, umap<int, MiMotor*>> MiMotor::instances;

MiMotor::MiMotor(const string& rid, int hid, string /*type*/, string port) :
    MotorDriver(rid, hid)
{
    this->port = port.back() - '0';
    instances[this->port][this->hid] = this;

    create_port(this->port);

    start();

    goal_helper(0, 0, 0, 0, 0);

    tx_thread = std::thread(&MiMotor::tx_loop, this);
}

MiMotor::~MiMotor()
{
    stop();
    if (tx_thread.joinable()) tx_thread.join();
    destroy_port(port);
}

void MiMotor::set_goal(double goal_pos, double goal_vel, double goal_tor)
{
    if (!std::isnan(goal_pos))
        goal_helper(goal_pos, 0, 0, kp, kd);
    else if (!std::isnan(goal_vel))
        goal_helper(0, goal_vel, 0, 0, kd);
    else // if (!std::isnan(goal_tor))
        goal_helper(0, 0, goal_tor, 0, 0);
}

void MiMotor::set_param(double p2v_kp, double /*p2v_ki*/, double p2v_kd,
    double /*v2t_kp*/, double /*v2t_ki*/, double /*v2t_kd*/)
{
    this->kp = p2v_kp;
    this->kd = p2v_kd;
}

std::tuple<double, double, double> MiMotor::get_state()
{
    return {
        position,
        velocity,
        torque
    };
}

void MiMotor::print_info()
{
    RCLCPP_INFO(rclcpp::get_logger("mi_driver"),
        "MiMotor rid %s hid %d created: kp %f, kd %f",
        rid.c_str(), hid,
        kp, kd);
}

void MiMotor::goal_helper(float pos, float vel, float tor, float kp , float kd)
{
    control_msg.ext_id.mode = 1;
    control_msg.ext_id.id = hid;
    control_msg.ext_id.data = float_to_uint(tor, T_MIN, T_MAX, 16);
    control_msg.ext_id.res = 0;
    control_msg.data[0] = float_to_uint(pos, P_MIN, P_MAX, 16) >> 8;
    control_msg.data[1] = float_to_uint(pos, P_MIN, P_MAX, 16);
    control_msg.data[2] = float_to_uint(vel, V_MIN, V_MAX, 16) >> 8;
    control_msg.data[3] = float_to_uint(vel, V_MIN, V_MAX, 16);
    control_msg.data[4] = float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8;
    control_msg.data[5] = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    control_msg.data[6] = float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8;
    control_msg.data[7] = float_to_uint(kd, KD_MIN, KD_MAX, 16);
}

void MiMotor::create_port(int port)
{
    if (can_drivers.find(port) == can_drivers.end())
    {
        can_drivers[port] = std::make_unique<CanDriver>(port);
        rx_threads[port] = std::thread(&MiMotor::rx_loop, port);
    }
}

void MiMotor::destroy_port(int port)
{
    if (instances[port].empty())
    {
        can_drivers.erase(port);
        if (rx_threads[port].joinable()) rx_threads[port].join();
    }
}

void MiMotor::tx(TxMsg msg)
{
    can_frame tx_frame;
    tx_frame.can_dlc = 8;
    tx_frame.can_id = *reinterpret_cast<uint32_t*>(&msg.ext_id) | CAN_EFF_FLAG; // id
    std::memcpy(tx_frame.data, msg.data.data(), 8); // data
    can_drivers[port]->send_frame(tx_frame);
}

void MiMotor::tx_loop()
{
    while (rclcpp::ok())
    {
        tx(control_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(TX_FREQ));
    }
}

int MiMotor::calc_id(const can_frame& frame)
{
    // the bit8 to bit15
    uint8_t id = (frame.can_id >> 8) & 0xff;
    return id;
}

void MiMotor::process_rx(const can_frame& frame)
{
    uint16_t buffer;
    auto& rx_data = frame.data;
    buffer = (rx_data[0] << 8 | rx_data[1]);
    position = ((float)buffer - 32767.5) / 32767.5 * 4 * 3.1415926f;

    buffer = (rx_data[2] << 8 | rx_data[3]);
    velocity = ((float)buffer - 32767.5) / 32767.5 * 30.0f;

    buffer = (rx_data[4] << 8 | rx_data[5]);
    torque = ((float)buffer - 32767.5) / 32767.5 * 12.0f;

    // buffer = (rx_data[6] << 8 | rx_data[7]);
    // temperature = (float)buffer/10.0f;
}

void MiMotor::rx_loop(int port)
{
    auto& can_driver = can_drivers[port];
    auto& can_frame = rx_frames[port];
    auto& instances = MiMotor::instances[port];
    while (rclcpp::ok())
    {
        can_driver->get_frame(can_frame);
        int id = calc_id(can_frame);

        // find corresponding instance and let it process
        if (instances.find(id) != instances.end())
            instances[id]->process_rx(can_frame);
    }
}

void MiMotor::start()
{
    TxMsg start_msg;
    start_msg.ext_id.mode = 3;
    start_msg.ext_id.id = hid;
    start_msg.ext_id.res = 0;
    start_msg.ext_id.data = MASTER_ID;
    start_msg.data.fill(0);
    tx(start_msg);
}

void MiMotor::stop()
{
    TxMsg stop_msg;
    stop_msg.ext_id.mode = 4;
    stop_msg.ext_id.id = hid;
    stop_msg.ext_id.res = 0;
    stop_msg.ext_id.data = MASTER_ID;
    stop_msg.data.fill(0);
    tx(stop_msg);
}

uint32_t MiMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

void MiMotor::curb(double &val, double limit)
{
    if (val > limit) val = limit;
    else if (val < -limit) val = -limit;
}