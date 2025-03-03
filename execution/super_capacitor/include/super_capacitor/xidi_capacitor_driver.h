#ifndef XIDI_CAPACITOR_DRIVER_H
#define XIDI_CAPACITOR_DRIVER_H

#include <thread>
#include <vector>
#include <unordered_map>
#include <stop_token>
#include <memory>
#include "super_capacitor/super_capacitor_base.h"
#include "meta_hardware/can_driver/can_driver.hpp"

namespace super_capacitor {

class XidiCapacitorDriver : public super_capacitor::SuperCapacitorBase
{
public:
    explicit XidiCapacitorDriver();

    void init(std::string can_interface) override;

    void set_target_power(double target_power) override;

    void set_referee_power(double referee_power) override;

    std::string get_device_name() override;

    std::unordered_map<std::string, double> get_state() override;

    ~XidiCapacitorDriver() override;
    
protected:
    double target_power_, referee_power_;
    std::vector<can_filter> can_filters_;
    std::unique_ptr<meta_hardware::CanDriver> can_driver_;
    void rx_loop(std::stop_token stop_token) override;
    std::unique_ptr<std::jthread> rx_thread_;
    double input_voltage_, capacitor_voltage_, input_current_, target_power_fb_;
    void tx() override;

};
}

#endif // XIDI_CAPACITOR_DRIVER_H