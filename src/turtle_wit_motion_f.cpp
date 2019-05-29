#include "ros/ros.h"
#include "serial/serial.h"
#include "turtle_wit_motion.h"
#include "unistd.h"
#include "string.h"
#include "tf/LinearMath/Quaternion.h"
int TurtleWitMotion::openPort(ros::NodeHandle& nod) {

    char choose = 'n';
    ros::Rate loop_openport(0.2);
    while(!ser_.isOpen()) {
        try {
 
            ROS_INFO_STREAM(port);
            
            ser_.setPort(port);
            ser_.setBaudrate(baudrate);

            serial::Timeout t_out = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(t_out);
            ser_.open();
            ROS_INFO_STREAM("Serial port initialized");
        }
        
        catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open port ");
            if (choose == 'n')
            {
            
                ROS_ERROR_STREAM("check port and try again \nDou you want try again?y or n?");
                choose = getchar();
            }
            
            if(choose == 'y') {

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
            else {
                ROS_ERROR_STREAM("exiting ,try ctrl C");
                exit(0);

            }
        }
        
       
    }
}


TurtleWitMotion::TurtleWitMotion(ros::NodeHandle& nod) {


    nod.param<std::string>("topic_read", topic_read, "read");
    nod.param<int>("Baudrate", baudrate);
    nod.param<std::string>("port", port, "/dev/pp");
    wit_pub_ = nod.advertise<sensor_msgs::Imu>(topic_read, 1);


}


void TurtleWitMotion::dataRead() {
    ros::Rate loop_rate(100);
    std::vector<uint8_t> buffer;
    int index = 0;
    uint8_t sum_cmp;
    while(ros::ok()) {
        if(ser_.available()){
            buffer.clear();
            ser_.read(buffer, 99);
            while(buffer.size())
            {
                sum_cmp = 0;
                if (buffer[0] == pack_first) {

                    if (buffer[1] == linear || buffer[1] == angular || buffer[1] == angle)
                    {
                        for (int j = 0; j < 10; ++j)
                        {
                            if(buffer.size() >= 11){
                                sum_cmp =sum_cmp + buffer[j];
                            }
                        }
                        if (sum_cmp == buffer[10] && buffer.size() >= 11)
                        {
                            time_wit_motion = ros::Time::now();
                            valueComplain(buffer);
                            buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+11);
                        }
                        else {

                            buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+2);
                            }
                    }
                    else {

                        buffer.begin() = buffer.erase(buffer.begin(), buffer.begin()+1);
               
                    }
                }

                else{
                    
                    buffer.begin() = buffer.erase(buffer.begin());
                }
  
            } 
        }
    

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
            // quate.setRPY(ang[0], ang[1], ang[2]);
            // result_imu.orientation.x = quate[0];
            // result_imu.orientation.z = quate[2];
            // result_imu.orientation.w = quate[3];        
            result_imu.linear_acceleration.x = a[0];
            result_imu.linear_acceleration.y = a[1];
            result_imu.linear_acceleration.z = a[2];
            result_imu.angular_velocity.x = w[0];

            result_imu.angular_velocity.y = w[1];
            result_imu.angular_velocity.z = w[2];
            result_imu.orientation.x = ang[0];
            result_imu.orientation.y = ang[1];
            result_imu.orientation.z = ang[2];   
        wit_pub_.publish(result_imu);

}
