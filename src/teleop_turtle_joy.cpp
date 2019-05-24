#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

class TeleopTurtleJoy
{
public:
	TeleopTurtleJoy(ros::NodeHandle& nh);
private:
	void teleopCallback(const sensor_msgs::Imu::ConstPtr& joy);
	ros::Publisher joy_pub_;
    ros::Subscriber joy_sub_;
};

TeleopTurtleJoy::TeleopTurtleJoy(ros::NodeHandle& nh) {
    ROS_INFO_STREAM("Turtle is already");
	joy_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	joy_sub_ = nh.subscribe<sensor_msgs::Imu>("read", 10, &TeleopTurtleJoy::teleopCallback, this);
}

void TeleopTurtleJoy::teleopCallback(const sensor_msgs::Imu::ConstPtr& joy) {	
    ROS_INFO_STREAM("Turtle is moving");
	geometry_msgs::Twist twist;
	
//  Displacement, yaw Angle, data not converted
	twist.angular.z = joy->angular_velocity.x/10;
	twist.linear.x = joy->linear_acceleration.x*10;
	joy_pub_.publish(twist);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtle_wit_motion");
	ros::NodeHandle nod;
	TeleopTurtleJoy teleopTurtleJoy(nod);
	ros::spin();
	return 0;
}