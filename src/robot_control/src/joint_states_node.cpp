#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

// Global variables for wheel velocities
double left_wheel_velocity = 0.0;
double right_wheel_velocity = 0.0;

// Global variables for integrated positions
double left_wheel_position = 0.0;
double right_wheel_position = 0.0;

// Last timestamp for integration
ros::Time last_time;

// Callback for left wheel velocity
void leftWheelCallback(const std_msgs::Float64::ConstPtr& msg) {
  left_wheel_velocity = msg->data;
}

// Callback for right wheel velocity
void rightWheelCallback(const std_msgs::Float64::ConstPtr& msg) {
  right_wheel_velocity = msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_states_node");
  ros::NodeHandle nh;

  // Subscribers to velocity feedback
  ros::Subscriber left_sub = nh.subscribe("/left_wheel_feedback", 10, leftWheelCallback);
  ros::Subscriber right_sub = nh.subscribe("/right_wheel_feedback", 10, rightWheelCallback);

  // Publisher for joint states
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  ros::Rate rate(10);  // 10 Hz
  last_time = ros::Time::now();

  while (ros::ok()) {
    // Calculate elapsed time since last loop
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;

    // Convert RPM to rad/s
    double left_velocity_rad = left_wheel_velocity * (2.0 * M_PI / 60.0);
    double right_velocity_rad = right_wheel_velocity * (2.0 * M_PI / 60.0);

    // Integrate velocity to compute position
    left_wheel_position += left_velocity_rad * dt;
    right_wheel_position += right_velocity_rad * dt;

    // Fill joint state message with both left and right wheel pairs
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = current_time;
    joint_msg.name = {
      "Back_Left_Wheel_Joint",
      "Front_Left_Wheel_Joint",
      "Back_Right_Wheel_Joint",
      "Front_Right_Wheel_Joint"
    };
    joint_msg.position = {
      left_wheel_position,
      left_wheel_position,
      right_wheel_position,
      right_wheel_position
    };

    // Publish joint states
    joint_pub.publish(joint_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
