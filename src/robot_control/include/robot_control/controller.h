#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class controller
{
public:
    controller(const ros::NodeHandle &, double radius, double separation);

private:
    void vel_callback(const geometry_msgs::Twist &);
    void joint_callback(const sensor_msgs::JointState &);
    ros::NodeHandle nh;
    ros::Subscriber vel_sub;
    ros::Subscriber joint_sub;
    ros::Publisher right_cmd_pub;
    ros::Publisher left_cmd_pub;
    ros::Publisher odom_pub;
    Eigen::Matrix2d speed_conversion;
    double wheel_radius;
    double wheel_seperation;
    double left_wheel_prev_pos;
    double right_wheel_prev_pos;
    ros::Time prev_time;
    double x;
    double y;
    double theta;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped transform_stamped;
};

#endif