#include <ros/ros.h>
#include "robot_control/controller.h"

int main (int argc, char **argv)
{
    ros::init(argc,argv,"controller_node");
    ros::NodeHandle nh;
    controller control(nh,0.17145/2,0.5417);
    ros::spin();
    return 0;
}