#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <iostream>
// #include <sensor_msgs/LaserScan.h>
#include <map>
#include "termios.h"

#include "lidar_driver/lidar_serial_driver.h"

#define             ANGLES_SCAN_HALF    180
#define             RANGES_LEN          (240 * 2 * ANGLES_SCAN_HALF / 360)
#define             RANGES_MAX          10.0
#define             RANGES_MIN          0.2
#define             PI                  3.1415926535

// void write_callback(const std_msgs::String::ConstPtr& msg){
//     ROS_INFO_STREAM("Writing to serial port" << msg->data);
//     ser.write(msg->data);
// }

// int main (int argc, char** argv){
//     ros::init(argc, argv, "DESerialPort_node");
//     ros::NodeHandle nh;
//     int StrengthThreshold, QualityThreshold, BitRate;
//     std::string COM;

//     if(!nh.getParam("StrengthThreshold", StrengthThreshold))
//         StrengthThreshold = 0;
//     if(!nh.getParam("QualityThreshold", QualityThreshold))
//         QualityThreshold = 0;
//     if(!nh.getParam("COM", COM))
//         COM = "/dev/ttyUSB0";
//     if(!nh.getParam("BitRate", BitRate))
//         BitRate = 460800;

//     ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
//     ros::Publisher read_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

//     try
//     {
//         ser.setPort(COM);
//         ser.setBaudrate(BitRate);
//         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//         ser.setTimeout(to);
//         ser.open();
//     }
//     catch (serial::IOException& e)
//     {
//         ROS_ERROR_STREAM("Unable to open port ");
//         return -1;
//     }

//     if(ser.isOpen()){
//         ROS_INFO_STREAM("Serial port open successed, now upload the laser data. ");
//     }else{
//         return -1;
//     }

//     DECOM::SensorDataItem SensorData;
//     sensor_msgs::LaserScan de_laser_scan;
//     std::map<float, float> laser_point;


//     //ros::Rate loop_rate(5);     //should > motor hz
//     while(ros::ok())
//     {
//         ros::spinOnce();

//         // ----------- get orignal data --------------------
//         //SensorData = DECOM::GetOrignalData(ser);
//         // ----------- get filtered data --------------------
//         SensorData = DECOM::GetFilteredData(ser, StrengthThreshold, QualityThreshold);
//         printf("%f %d %d %d %f\n", SensorData.Dist, SensorData.IndexInCycle, SensorData.Strength, SensorData.Quality_Score, SensorData.Angle);

//         if(((SensorData.Angle < PI) && (SensorData.Angle > -PI)))
//             laser_point.insert(std::make_pair(SensorData.Angle, SensorData.Dist));
//         //ROS_INFO_STREAM( laser_point.size() );
//         if ( laser_point.size() == RANGES_LEN - 1)     //note: a whole circle only once, may be less 1
//         {
//             de_laser_scan.ranges.clear();

//             std::map<float,float>::iterator it;
//             it = laser_point.begin();

//             de_laser_scan.angle_min = laser_point.begin()->first;//start of the scan angle

//             de_laser_scan.range_min = RANGES_MIN;
//             de_laser_scan.range_max = RANGES_MAX;

//             //find max and min to threthold distance
//             for ( ; it != laser_point.end(); it ++ )
//             {
//                 de_laser_scan.angle_max = it->first;
//             //threthold_range_max
//                 if( it->second > de_laser_scan.range_max )
//                 {
//                     it->second = de_laser_scan.range_max;
//                 }
//                 //threthold_range_min
//                 if( it->second < de_laser_scan.range_min )
//                 {
//                     it->second = de_laser_scan.range_min;
//                 }

//                 de_laser_scan.ranges.push_back(it->second);
//                 //de_laser_scan.intensities.push_back(0);
//             }

//             de_laser_scan.angle_increment = (de_laser_scan.angle_max - de_laser_scan.angle_min) / RANGES_LEN;
//             //time_increment
//             de_laser_scan.time_increment = 1/3/240;
//             //header-stamp
//             de_laser_scan.header.stamp = ros::Time::now();
//             //header-frame_id:1-gloable, 0-no frame
//             de_laser_scan.header.frame_id = "laser";
//             //publish scan
//             read_pub.publish(de_laser_scan);
//             //clear
//             //ROS_INFO_STREAM("PUBLISHED");
//             //de_laser_scan.ranges.clear();
//             laser_point.clear();
//         }
//         //loop_rate.sleep();

//     }
// }

class LidarDriver : public rclcpp::Node
{
public:

private:
};

int main()
{
    rclcpp::init(0, nullptr);
    return 0;
}