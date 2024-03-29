#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>

#include "ros_drive_controller/i2c.h"

class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
    MyRobotHWInterface(ros::NodeHandle& nh);
    ~MyRobotHWInterface();

    void init();
    void update(const ros::TimerEvent& te);
    void read();
    void write(ros::Duration elapsed_time);

protected:

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;


    joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface_;

    int num_joints_;


    std::string joint_name_;
    double joint_position_;
    double joint_velocity_;
    double joint_effort_;
    double joint_position_command_;
    double joint_effort_command_;
    double joint_velocity_command_;

    int motor_pos;
    i2c::I2C my_motor = i2c::I2C(1,10);

    ros::NodeHandle nh_;
    ros::Timer async_time;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

private:
    int motor_prev_cmd = 0;

};