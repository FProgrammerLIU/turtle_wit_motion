#include "ros/ros.h"
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include "turtle_wit_motion.h"




int main(int argc, char** argv) {
    ros::init(argc, argv, "mition_serial");
    ros::NodeHandle nod;
    TurtleWitMotion turtleWitMotion(nod);
    return 0;
}