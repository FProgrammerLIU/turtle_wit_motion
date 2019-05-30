#include "ros/ros.h"
#include "serial/serial.h"
#include "turtle_wit_motion.h"
#include "unistd.h"
#include "string.h"
#include "tf/LinearMath/Quaternion.h"
bool TurtleWitMotion::openPort() {

    char choose = 'n';
    ros::Rate loop_openport(0.2);
    while(!ser_.isOpen()) {
        try {
            ROS_INFO_STREAM(port);    
            ser_.setPort(port);
            ser_.setBaudrate(115200);
            serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(t_out);
            ser_.open();
            ROS_INFO_STREAM("Serial port initialized");
        }
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port ");
            ROS_ERROR_STREAM("Try again ");
            sleep(1);    
            ROS_ERROR_STREAM("5");
            sleep(1);    
            ROS_ERROR_STREAM("4");
            sleep(1);    
            ROS_ERROR_STREAM("3");
            sleep(1);    
            ROS_ERROR_STREAM("2");
            sleep(1);    
            ROS_ERROR_STREAM("1");
            loop_openport.sleep(); 
        }        
    }
}


TurtleWitMotion::TurtleWitMotion(ros::NodeHandle& nod) {

    nod.param<std::string>("topic_read", topic_read, "read");
    nod.param<int>("Baudrate", baudrate, 9600);
    nod.param<std::string>("port", port, "/dev/pp");
    wit_pub_ = nod.advertise<sensor_msgs::Imu>(topic_read, 1);


}


void TurtleWitMotion::dataRead() {
    ros::Rate loop_rate(100);
    std::vector<uint8_t> buffer;
    int index_data = 0;
    uint8_t sum_cmp;
    while(ros::ok()) {
        if(ser_.available()) {
            buffer.clear();
            ser_.read(buffer, 99);
            while(buffer.size()) {
                sum_cmp = 0;
                if (buffer[index_data] == pack_first) {
                    if (buffer[index_data+1]==linear || buffer[index_data+1]==angular || buffer[index_data+1]==angle) {
                        if (buffer.size()>=11)
                        {
                            for (int j=0; j<10; ++j) {
                                sum_cmp =sum_cmp + buffer[index_data+j];                           
                            }
                            index_data += 11; 
                            if (sum_cmp == buffer[index_data-1]) {
                                time_wit_motion = ros::Time::now();
                                valueComplain(buffer);
                                buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+index_data);
                                index_data = 0;
                            }
                        }
                        else {
                            buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+buffer.size());
                        }
                    }
                    else {
                        index_data += 1;
                    } 
                }
                else {
                    index_data += 1;
                }
            }
        }
    index_data = 0;
    ros::spinOnce();
    loop_rate.sleep();
    }
}

void TurtleWitMotion::motioncallBack(const sensor_msgs::Imu::ConstPtr& imu) {

}

void TurtleWitMotion::valueComplain(std::vector<uint8_t> &vComplain) {
    sensor_msgs::Imu result_imu; 
    tf::Quaternion quate;
    result_imu.header.stamp = time_wit_motion;
    result_imu.header.frame_id = "id_wit_motion";
//  Data analysis
    switch(vComplain[1]) {
    
        case 0x51:        
            a[0] = (short(vComplain[3]<<8|vComplain[2]))/32768.0*16.0*9.8;
            a[1] = (short(vComplain[5]<<8|vComplain[4]))/32768.0*16.0*9.8;
            a[2] = (short(vComplain[7]<<8|vComplain[6]))/32768.0*16.0*9.8;
            break;
        case 0x52:        
            w[0] = (short(vComplain[3]<<8|vComplain[2]))/32768.0*2000.0*PI/180.0;
            w[1] = (short(vComplain[5]<<8|vComplain[4]))/32768.0*2000.0*PI/180.0;
            w[2] = (short(vComplain[7]<<8|vComplain[6]))/32768.0*2000.0*PI/180.0;
            break;
        case 0x53:
            ang[0] = (short(vComplain[3]<<8|vComplain[2]))/32768.0*2000.0*PI/180.0;
            ang[1] = (short(vComplain[5]<<8|vComplain[4]))/32768.0*2000.0*PI/180.0;
            ang[2] = (short(vComplain[7]<<8|vComplain[6]))/32768.0*2000.0*PI/180.0;
            break;
        }      
            quate.setRPY(ang[0], ang[1], ang[2]);
            result_imu.orientation.x = quate[0];
            result_imu.orientation.y = quate[1];
            result_imu.orientation.z = quate[2];
            result_imu.orientation.w = quate[3];        
            result_imu.linear_acceleration.x = a[0];
            result_imu.linear_acceleration.y = a[1];
            result_imu.linear_acceleration.z = a[2];
            result_imu.angular_velocity.x = w[0];
            result_imu.angular_velocity.y = w[1];
            result_imu.angular_velocity.z = w[2];
          
            result_imu.orientation_covariance[0] = -1;
            result_imu.angular_velocity_covariance[0] = -1;
            result_imu.linear_acceleration_covariance[0] = -1;
        wit_pub_.publish(result_imu);
}
