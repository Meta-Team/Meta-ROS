#ifndef GYRO_DRIVER_H
#define GYRO_DRIVER_H

class GyroDriver
{
public:
    float yaw;
    float roll;
    float pitch;

    void get_frame();

    void process_rx();
};

#endif // GYRO_DRIVER_H