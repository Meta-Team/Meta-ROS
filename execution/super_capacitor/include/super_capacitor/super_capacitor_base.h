#ifdef SUPER_CAPACITOR_BASE_H
#define SUPER_CAPACITOR_BASE_H

namespace super_capacitor
{

class SuperCapcitorBase : public SuperCapacitorBaseInterface
{
public:
    SuperCapacitorBase() = default;

    virtual void set_target_power(float target_power) = 0;

    virtual void set_referee_power(float referee_power) = 0;

    virtual void can_tx() = 0;

    virtual void can_rx() = 0;

    // virtual ~SuperCapcitorBase() = default;
};

}


#endif // SUPER_CAPACITOR_BASE_H