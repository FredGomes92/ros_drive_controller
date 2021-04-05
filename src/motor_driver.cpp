#include <ros/ros.h>
#include "ros_drive_controller/i2c.h"
#include <string>

using namespace std;
using namespace i2c;

void GetResponse(string &ss, I2C *motor)
{
    uint8_t buff[3] = {0};

    motor->readBytes(buff, 5);

    ROS_INFO("Data received: ");

    for (int i = 0; i < 5; i++)
    {
        ss += buff[i];

        ROS_INFO("Byte received : %d", buff[i]);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle _nh;

    I2C motor(1, 10); // Bus 1 , address 10

    string ss;

    int vel = 300; // º/s
    int pos = 102;

    uint8_t buff[4];

    buff[0] = vel >> 8; //MSB
    buff[1] = vel; // LSB
    buff[2] = pos >> 8; // MSB
    buff[3] = pos; // LSB

    ROS_INFO ("I2c master has started!");

    while( ros::ok())
    {
        ROS_INFO("Writting data ...");
        ROS_INFO("vel: buff[0]: %d, buff[1]= %d", buff[0], buff[1]);
        ROS_INFO("pos: buff[2]: %d, buff[3]= %d", buff[2], buff[3]);

        motor.writeData(buff,4);

        ros::Duration(2).sleep();

        // GetResponse(ss, &motor);

        ROS_INFO("Received data: %s", ss);

    }

    return 0;
}