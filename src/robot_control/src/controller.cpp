#include "robot_control/controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

controller::controller(const ros::NodeHandle &n_h, double radius, double seperation) : nh(n_h),
                                                                                       wheel_radius(radius),
                                                                                       wheel_seperation(seperation),
                                                                                       left_wheel_prev_pos(0.0),
                                                                                       right_wheel_prev_pos(0.0),
                                                                                       x(0),
                                                                                       y(0),
                                                                                       theta(0)

{
    prev_time = ros::Time::now();

    right_cmd_pub = nh.advertise<std_msgs::Float64>("right_wheel_command", 10);
    left_cmd_pub = nh.advertise<std_msgs::Float64>("left_wheel_command", 10);
    odom_pub=nh.advertise<nav_msgs::Odometry>("odom",10);

    vel_sub = nh.subscribe("/cmd_vel", 1000, &controller::vel_callback, this);
    joint_sub = nh.subscribe("/joint_states", 1000, &controller::joint_callback, this);
    
    odom_msg.header.frame_id="odom";
    odom_msg.child_frame_id="base_footprint";
    odom_msg.pose.pose.orientation.x=0.0;
    odom_msg.pose.pose.orientation.y=0.0;
    odom_msg.pose.pose.orientation.z=0.0;
    odom_msg.pose.pose.orientation.w=1.0;

    // transform_stamped.header.frame_id="odom";
    // transform_stamped.child_frame_id="base_footprint";

    speed_conversion << radius / 2, radius / 2, radius / seperation, -radius / seperation;
}

void controller::vel_callback(const geometry_msgs::Twist &msg)
{
    Eigen::Vector2d robot_speed(msg.linear.x, msg.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion.inverse() * robot_speed;

    std_msgs::Float64 right_speed;
    std_msgs::Float64 left_speed;

    right_speed.data = wheel_speed.coeff(0);
    left_speed.data = wheel_speed.coeff(1);

    right_cmd_pub.publish(right_speed);
    left_cmd_pub.publish(left_speed);
}

void controller::joint_callback(const sensor_msgs::JointState &state)
{
    double dp_left = state.position.at(0) - left_wheel_prev_pos;
    double dp_right = state.position.at(1) - right_wheel_prev_pos;
    double dt = (state.header.stamp - prev_time).toSec();

    left_wheel_prev_pos = state.position.at(0);
    right_wheel_prev_pos = state.position.at(1);
    prev_time = state.header.stamp;

    double fi_left = dp_left / dt;
    double fi_right = dp_right / dt;

    double linear = (wheel_radius * fi_right + wheel_radius * fi_left) / 2;
    double angular = (wheel_radius * fi_right - wheel_radius * fi_left) / wheel_seperation;

    double d_s = (wheel_radius * dp_right + wheel_radius * dp_left) / 2;
    double d_theta = (wheel_radius * dp_right - wheel_radius * dp_left) / wheel_seperation;
    theta += d_theta;
    x += d_s * cos(theta);
    y += d_s * sin(theta);

    tf2::Quaternion q;
    q.setRPY(0.0,0.0,theta);
    odom_msg.pose.pose.orientation.x=q.x();
    odom_msg.pose.pose.orientation.y=q.y();
    odom_msg.pose.pose.orientation.z=q.z();
    odom_msg.pose.pose.orientation.w=q.w();
    odom_msg.header.stamp=ros::Time::now();
    odom_msg.pose.pose.orientation.x=x;
    odom_msg.pose.pose.orientation.y=y;
    odom_msg.twist.twist.linear.x=linear;
    odom_msg.twist.twist.angular.z=angular;
    odom_pub.publish(odom_msg);


    // transform_stamped.transform.translation.x=x;
    // transform_stamped.transform.translation.x=y;
    // transform_stamped.transform.rotation.x=q.getX();
    // transform_stamped.transform.rotation.y=q.getY();
    // transform_stamped.transform.rotation.z=q.getZ();
    // transform_stamped.transform.rotation.w=q.getW();
    // transform_stamped.header.stamp=ros::Time::now();

    // static tf2_ros::TransformBroadcaster br;
    // br.sendTransform(transform_stamped);

    // ROS_INFO_STREAM("linear: " << linear << " angular: " << angular);
    // ROS_INFO_STREAM("x: "<<x);
    // ROS_INFO_STREAM("y: "<<y);
    // ROS_INFO_STREAM("theta: "<<theta);

}