#ifndef SUPER_CAPACITOR_BASE_H
#define SUPER_CAPACITOR_BASE_H
#include <unordered_map>
#include <string>
#include <linux/can.h>
#include <stop_token>

namespace super_capacitor
{
class SuperCapacitorBase
{
public:
    SuperCapacitorBase() = default;

    virtual void init(std::string can_interface) = 0;

    virtual void set_target_power(double target_power) = 0;

    virtual void set_referee_power(double referee_power) = 0;

    virtual std::unordered_map<std::string, double> get_state() = 0;

    virtual std::string get_device_name() = 0;

    virtual ~SuperCapacitorBase() = default;

    virtual void tx() = 0;
protected:
    virtual void rx_loop(std::stop_token stop_token) = 0;
};
}


#endif // SUPER_CAPACITOR_BASE_H