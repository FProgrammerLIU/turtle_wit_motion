#ifndef __TURTLE_WIT_MOTION_INCLUDE_TURTLE_WIT_MOTION_H__
#define __TURTLE_WIT_MOTION_INCLUDE_TURTLE_WIT_MOTION_H__

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "vector"
#include "sensor_msgs/Joy.h"
class TurtleWitMotion {
    
public:
    
    TurtleWitMotion(ros::NodeHandle& nod_);
    void teleopTurtleJoy(ros::NodeHandle& nod_);

private:
    serial::Serial ser_;
    void motioncallBack(const sensor_msgs::Imu::ConstPtr& imu);
    void valueComplain(std::vector<uint8_t> &vComplain);
    ros::Publisher wit_pub_,joy_pub_;
    ros::Subscriber wit_sub_,joy_sub_;

};
#endif