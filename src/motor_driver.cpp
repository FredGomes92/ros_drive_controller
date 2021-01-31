#include <ros/ros.h>
#include "ros_drive_controller/i2c.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "motor_driver");

    ros::NodeHandle _nh;


    I2C motor(1, 10); // Bus 1 , address 10

    uint8_t buff[1] = {10};

    while( ros::ok())
    {

        motor.writeData(buff,1);

        ros::Duration(2).sleep();

    }

    return 0;


}