#include <robot_controller/robot_hardware_interface.h>

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    pub = nh_.advertise<rospy_tutorials::Floats>("joints_to_arduino", 10);
    client = nh_.serviceClient<robot_controller::Floats_array>("read_joint_state");

    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface()
{
}

void ROBOTHardwareInterface::init()
{

    for (int i = 0; i < 2; i++)
    {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create velocity joint interface
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // Create Joint Limit interface
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
        joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
        velocityJointSaturationInterface.registerHandle(jointLimitsHandle);
    }

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read()
{
    joint_read.request.req = 1.0;

    if (client.call(joint_read))
    {
        left_motor_pos = angles::from_degrees(joint_read.response.res[1]);
        joint_position_[1] = left_motor_pos;

        right_motor_pos = angles::from_degrees(joint_read.response.res[0]);
        joint_position_[0] = right_motor_pos;
        ROS_INFO("left wheel Pos : %.2f, right wheel Pos : %.2f", joint_position_[1], joint_position_[0]);
    }
    else
    {
        joint_position_[0] = 0;
        joint_position_[1] = 0;
    }
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time)
{

    velocityJointSaturationInterface.enforceLimits(elapsed_time);

    joints_pub.data.clear();

    joint_send_vel_[0] = (double)angles::to_degrees(joint_velocity_command_[0]);
    joints_pub.data.push_back(joint_send_vel_[0]);

    joint_send_vel_[1] = (double)angles::to_degrees(joint_velocity_command_[1]);
    joints_pub.data.push_back(joint_send_vel_[1]);

    ROS_INFO("left wheel vel : %.2f, right wheel vel : %.2f", joint_send_vel_[0], joint_send_vel_[1]);
    pub.publish(joints_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(4);
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    // spinner.start();
    spinner.spin();
    // ros::spin();
    return 0;
}