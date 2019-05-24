#include "ros/ros.h"
#include "vector"
#include "serial/serial.h"
#include "turtle_wit_motion.h"
#include "sensor_msgs/Imu.h"
TurtleWitMotion::TurtleWitMotion(ros::NodeHandle& nod) {
    wit_sub_ = nod.subscribe("write", 1000, &TurtleWitMotion::motioncallBack,this);
    wit_pub_ = nod.advertise<sensor_msgs::Imu>("read", 1);

    try {
        ser_.setPort("/dev/ttyUSB0");
        ser_.setBaudrate(115200);
        serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
        ser_.setTimeout(t_out);
        ser_.open();
    }
    catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
    }

    if(ser_.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else {
    }

    ros::Rate loop_rate(100);
    while(ros::ok()) {
        if(ser_.available()){
  //        Read serial data  
            std::vector<uint8_t> buffer_decimal;
            ser_.read(buffer_decimal, ser_.available());
            valueComplain(buffer_decimal);
        }
    
    ros::spinOnce();
    loop_rate.sleep();
    }
}
void TurtleWitMotion::motioncallBack(const sensor_msgs::Imu::ConstPtr& imu) {

}
void TurtleWitMotion::valueComplain(std::vector<uint8_t> &vComplain) {
    uint8_t linear = 0x51, angular = 0x52, tengle = 0x53, msg = 0x55;
    short a[3], w[3], angle[3];
    sensor_msgs::Imu result_imu; 
//  Data analysis
    if (vComplain[0] == 0x55)
    {
        if (vComplain[1] == linear)
        {
            a[0] = (short(vComplain[3]<<8|vComplain[2]))/32768.0*16;
            a[1] = (short(vComplain[5]<<8|vComplain[4]))/32768.0*16;
            a[2] = (short(vComplain[7]<<8|vComplain[6]))/32768.0*16;
            // ROS_INFO("Read:0x%x", vComplain[2]);
        }

        if (vComplain[12] == angular)
        {
            w[0] = (short(vComplain[14]<<8|vComplain[13]))/32768.0*2000;
            w[1] = (short(vComplain[16]<<8|vComplain[15]))/32768.0*2000;
            w[2] = (short(vComplain[18]<<8|vComplain[17]))/32768.0*2000;
            // ROS_INFO("Read:0x%x", vComplain[14]);
        }

        if (vComplain[23] == tengle)
        {   angle[0] = (short(vComplain[25]<<8|vComplain[24]))/32768.0*180;
            angle[1] = (short(vComplain[27]<<8|vComplain[26]))/32768.0*180;
            angle[2] = (short(vComplain[29]<<8|vComplain[28]))/32768.0*180;
            // ROS_INFO("Read:0x%x", vComplain[25]);
        }
          
        // The data is stored in the IMU.msg and published
        result_imu.linear_acceleration.x = a[0];
        result_imu.linear_acceleration.y = a[1];
        result_imu.linear_acceleration.z = a[2];

        result_imu.angular_velocity.x = w[0];
        result_imu.angular_velocity.y = w[1];
        result_imu.angular_velocity.z = w[2];

        result_imu.orientation.x = angle[0];
        result_imu.orientation.y = angle[1];
        result_imu.orientation.z = angle[2];

        
        wit_pub_.publish(result_imu);

    }
}