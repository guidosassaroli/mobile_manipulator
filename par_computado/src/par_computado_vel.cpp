#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <math.h>
#include <geometry_msgs/TwistStamped.h>


void callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint)
{
    hardware_interface::JointHandle joint_;
    double Wn = 1;
    double Xi = 1;
    trajectory_msgs::JointTrajectory traj;
    std::vector<hardware_interface::JointHandle> joints_;
    
    //here we have to write the equations of the par_computado 

    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "par_computado");

    ros::NodeHandle n;
    ros::NodeHandle n_local("~");

    ros::Publisher motor_vel = n.advertise<geometry_msgs::TwistStamped>("motor_vel", 1000);

    // current_task_publisher =
    // n.advertise<integration::ScheduleExecutorTaskStatus>("/schedule_executor/task_status", 10, true);   

    ros::Publisher motor_velocity = n.advertise<geometry_msgs::TwistStamped>("motor_velocity", 1000);

    ros::Subscriber traj_sub;

    //callback

    ros::spin();

    return 0;
}