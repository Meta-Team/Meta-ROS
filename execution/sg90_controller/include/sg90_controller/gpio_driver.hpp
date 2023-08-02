#include <stdio.h>
#include <string>

class GPIODriver
{
public:
    void init_port(int port)
    {
        p = fopen( "/sys/class/gpio/export", "w" );
        fprintf( p, "%d", port );
        fclose( p );
    }
    void set_output(int port)
    {
        std::string file_name = "/sys/class/gpio/gpio" + std::to_string(port) + "/direction";
        p = fopen(file_name.c_str(), "w" );
        fprintf( p, "out" );
        fclose( p );
    }
    void set_high(int port)
    {
        std::string file_name = "/sys/class/gpio/gpio" + std::to_string(port) + "/value";
        p = fopen(file_name.c_str(), "w" );
        fprintf( p, "%d", 1 );
        fclose( p );
    }
    void set_low(int port)
    {
        std::string file_name = "/sys/class/gpio/gpio" + std::to_string(port) + "/value";
        p = fopen(file_name.c_str(), "w" );
        fprintf( p, "%d", 0 );
        fclose( p );
    }
    void end_port(int port)
    {
        std::string file_name = "/sys/class/gpio/unexport";
        p = fopen(file_name.c_str(), "w" );
        fprintf( p, "%d", port );
        fclose( p );
    }

private:
    FILE *p;
};