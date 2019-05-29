#include "ros/ros.h"
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include "turtle_wit_motion.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_wit_motion");
    ros::NodeHandle nod;
    // TurtleWitMotion turtleWit();
    TurtleWitMotion turtleWitMotion(nod);
    turtleWitMotion.openPort(nod);
    turtleWitMotion.dataRead();
    return 0;
}