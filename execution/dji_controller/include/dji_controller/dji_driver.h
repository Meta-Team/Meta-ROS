enum MotorType
{
    M3508,
    M2006,
    M6020,
};

class DjiDriver
{
private:
    int motor_id;
    MotorType motor_type;

    float present_pos;
    float former_pos;
    float present_vel;
    float former_vel;
    float current;

public:
    void update_pos(float pos);
    void update_vel(float vel);
    float vel2current(float goal_vel);
    float pos2current(float goal_pos);
    static void set_current(float &current, float goal);
};