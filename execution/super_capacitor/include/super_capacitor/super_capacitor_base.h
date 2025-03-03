#ifdef SUPER_CAPACITOR_BASE_H
#define SUPER_CAPACITOR_BASE_H
#include <unordered_map>
#include <string>
#include <linux/can.h>

namespace super_capacitor_base{
class SuperCapcitorBase : public SuperCapacitorBaseInterface
{
public:
    SuperCapacitorBase(std::string can_interface) = default;

    virtual void set_target_power(float target_power) = 0;

    virtual void set_referee_power(float referee_power) = 0;

    virtual std::unordered_map<std::string, double> get_state();

    std::string get_device_name();

    ~SuperCapcitorBase() = default;

private:
    virtual void tx() = 0;

    virtual void rx_loop(std::stop_token stop_token) = 0;
};
}


#endif // SUPER_CAPACITOR_BASE_H