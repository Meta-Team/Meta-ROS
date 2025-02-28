#include "super_capacitor_base.h"

namespace super_capacitor
{
class Xidi_CapacitorDriver : public SuperCapacitorBase
{
public:
    Xidi_CapacitorDriver() = default;

    void set_target_power(float target_power);

    void set_referee_power(float referee_power) = 0;

    void can_tx() = 0;

    void can_rx() = 0;

    // virtual ~Xidi_CapacitorDriver() = default;

};
}