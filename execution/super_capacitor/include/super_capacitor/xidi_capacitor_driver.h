#include <thread>

#include "super_capacitor_base/super_capacitor_base.h"

namespace super_capacitor_plugin{
class XidiCapacitorDriver : public SuperCapacitorBase
{
public:
    XidiCapacitorDriver(std::string can_interface);

    void set_target_power(double target_power);

    void set_referee_power(double referee_power);

    std::string get_device_name();

    void ~XidiCapacitorDriver();
    
private:
    // CAN driver
    std::vector<can_filter> can_filters_;
    std::unique_ptr<CanDriver> can_driver_;
    void rx_loop(std::stop_token stop_token);
    std::unique_ptr<std::jthread> rx_thread_;
    double input_voltage_, capacitor_voltage_, input_current_, target_power_;
    void tx();

};
}