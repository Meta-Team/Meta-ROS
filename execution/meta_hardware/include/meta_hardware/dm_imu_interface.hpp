#ifndef META_HARDWARE__DM_IMU_INTERFACE__
#define META_HARDWARE__DM_IMU_INTERFACE__
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "meta_hardware/dm_imu_interface_utils.hpp"
#include "meta_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "serial_driver/serial_driver.hpp"

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

namespace meta_hardware
{
    constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
    class DMIMUInterface : public hardware_interface::SensorInterface
    {
    public:
        ~DMIMUInterface() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;


    private:
        typedef struct
        {
            float accx;
            float accy;
            float accz;
            float gyrox;
            float gyroy;
            float gyroz;
            float roll;
            float pitch;
            float yaw;
        } dm_imu_data_t;
        typedef struct
        {
            double orientation_x;
            double orientation_y;
            double orientation_z;
            double orientation_w;
            double angular_velocity_x;
            double angular_velocity_y;
            double angular_velocity_z;
            double linear_acceleration_x;
            double linear_acceleration_y;
            double linear_acceleration_z;
        } dm_imu_state_t;
#pragma pack(1)
        typedef struct
        {
            uint8_t FrameHeader1;
            uint8_t flag1;
            uint8_t slave_id1;
            uint8_t reg_acc;
            uint32_t accx_u32;
            uint32_t accy_u32;
            uint32_t accz_u32;
            uint16_t crc1;
            uint8_t FrameEnd1;

            uint8_t FrameHeader2;
            uint8_t flag2;
            uint8_t slave_id2;
            uint8_t reg_gyro;
            uint32_t gyrox_u32;
            uint32_t gyroy_u32;
            uint32_t gyroz_u32;
            uint16_t crc2;
            uint8_t FrameEnd2;

            uint8_t FrameHeader3;
            uint8_t flag3;
            uint8_t slave_id3;
            uint8_t reg_euler; // r-p-y
            uint32_t roll_u32;
            uint32_t pitch_u32;
            uint32_t yaw_u32;
            uint16_t crc3;
            uint8_t FrameEnd3;
        } imu_recv_frame_t;
#pragma pack()
        void get_imu_data_thread(std::stop_token s);
        void init_imu_serial(std::string& dm_imu_devpath);

        void enter_setting_mode();
        void turn_on_accel();
        void turn_on_gyro();
        void turn_on_euler();
        void turn_off_quat();
        void set_output_1000HZ();
        void save_imu_para();
        void exit_setting_mode();
        void restart_imu();

        imu_recv_frame_t receive_data_{};
        dm_imu_data_t data_{};
        dm_imu_state_t dm_imu_state_;
        std::unique_ptr<std::jthread> rx_thread_;
        std::unique_ptr<SerialDriver> dm_imu_serial_driver_;
        std::unique_ptr<IoContext> dm_imu_ctx_;

        static constexpr uint32_t baudrate = 921600;
        std::string sensor_name_, tty_devpath_;
        std::unique_ptr<rclcpp::Logger> logger_;
    };

} // namespace meta_hardware
#endif // META_HARDWARE__DM_IMU_INTERFACE__