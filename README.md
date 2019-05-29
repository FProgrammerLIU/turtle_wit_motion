# turtle_wit_motion
## usage
Receiving serial messages, controlling turtlesim
## environment
Ubuntu16.04 ROS-kinetic C++5.1 sensor type:WT61C
## prerequisites
ros apt-get install ros-kinetic-serial
***
ros apt-get install ros-kinetic-turtlesim
## run
roslaunch turtle_wit_motion turtle_wit_motion.launch
## How to use
Position the sensor horizontally and rotate along the X-axis and Y-axis to control turtlexim
## Quaternion conversion
Comment the Angle conversion code and uncomment the quaternion section code
## note
If you don't want to keep changing your serial name and permissions, put the rules file into your /lib/udev/rules.d/ or /etc/udev/rules.d/ directory.



