#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ur5e_moveit_example");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello World");
}