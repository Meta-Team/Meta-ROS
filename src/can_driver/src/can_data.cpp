#include "can_data.h"
#include <can_interfaces/msg/detail/mit_data__struct.hpp>
#include "can_interfaces/msg/detail/pos_vel_data__struct.hpp"
#include "can_interfaces/msg/detail/vel_data__struct.hpp"

void CanData::mit_interpreter(can_frame &frame, can_interfaces::msg::MITData &data)
{
    int frame_id = data.id;
    frame.can_id = frame_id;
    frame.can_dlc = 8;
    frame.data[0] = data.p_des;
    frame.data[1] = data.p_des;
    frame.data[2] = data.v_des;
    frame.data[3] = data.v_des;
    frame.data[4] = data.kp;
    frame.data[5] = data.kd;
    frame.data[6] = data.t_ff;
    frame.data[7] = data.t_ff; // to be changed
}

void CanData::posvel_interpreter(can_frame &frame, can_interfaces::msg::PosVelData &data)
{
    int frame_id = data.id + 0x100;
    frame.can_id = frame_id;
    frame.can_dlc = 8;
    frame.data[0] = data.p_des;
    frame.data[1] = data.p_des;
    frame.data[2] = data.p_des;
    frame.data[3] = data.p_des;
    frame.data[4] = data.v_des;
    frame.data[5] = data.v_des;
    frame.data[6] = data.v_des;
    frame.data[7] = data.v_des; // to be changed
}

void CanData::vel_interpreter(can_frame &frame, can_interfaces::msg::VelData &data)
{  
    int frame_id = data.id + 0x200;
    frame.can_id = frame_id;
    frame.can_dlc = 8;
    frame.data[0] = data.v_des;
    frame.data[1] = data.v_des;
    frame.data[2] = data.v_des;
    frame.data[3] = data.v_des;
    frame.data[4] = data.v_des;
    frame.data[5] = data.v_des;
    frame.data[6] = data.v_des;
    frame.data[7] = data.v_des;
}

can_interfaces::msg::Feedback CanData::feedback_interpreter(const can_frame &frame_read)
{
    can_interfaces::msg::Feedback feedback;
    feedback.mst_id = frame_read.can_id;
    feedback.id = frame_read.data[0];
    feedback.pos = frame_read.data[1];
    feedback.vel = frame_read.data[2];
    feedback.tor = frame_read.data[3];
    feedback.t_mos = frame_read.data[4];
    feedback.t_rotor = frame_read.data[5];
    return feedback;
}