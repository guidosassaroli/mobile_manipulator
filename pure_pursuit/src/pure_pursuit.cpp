#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <kdl/frames.hpp>

#include <pure_pursuit/PurePursuitConfig.h>

#include <tf/tf.h>

#include <iostream>
#include <iterator>
#include <random>
#include <cstdlib>

using std::string;

class PurePursuit
{
public:

  //! Constructor
  PurePursuit();

  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(nav_msgs::Odometry odom);

  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);

  //! Compute transform that transforms a pose into the robot frame (robot_footprint)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);
  
  //Function for computing eucledian distances in the x-y plane.
   double distance(double x1, double y1, double x2, double y2)  
  {   
    return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2));
  }

  //! Run the controller.
  void run();
  
private:

  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  // Generic control variables
  double v_max_, v_, w_max_;

  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_;
  // string acker_frame_id_;


  
};

PurePursuit::PurePursuit() : ld_(0.7), v_max_(0.2), v_(v_max_), w_max_(0.4), pos_tol_(0.1), idx_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("robot_footprint"),
                             lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("lookahead_distance", ld_, 0.7);
  nh_private_.param<double>("linear_velocity", v_, 0.2);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 0.4);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "robot_footprint");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;
  
  // /move_base/TrajectoryPlannerROS/local_plan
  sub_path_ = nh_.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1, &PurePursuit::receivePath, this);
  sub_odom_ = nh_.subscribe("/odometry/filtered", 1, &PurePursuit::computeVelocities, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/ommp_velocity_controller/cmd_vel", 1);

 
}

void PurePursuit::computeVelocities(nav_msgs::Odometry odom)
{
  double x_robot = odom.pose.pose.position.x;
  double y_robot = odom.pose.pose.position.y;

  tf::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

  // Get the current robot pose
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.poses.size(); idx_++)
    {
      if (distance(path_.poses[idx_].pose.position.x, path_.poses[idx_].pose.position.y, tf.transform.translation.x, tf.transform.translation.y) > ld_)
      {

        // Transformed lookahead to robot_footprint frame is lateral error
        KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);

        break;
      }
    }

    if (!path_.poses.empty() && idx_ >= path_.poses.size())
    {
      // We are approaching the goal, which is closer than ld

      // This is the pose of the goal w.r.t. the robot_footprint frame
      KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_ = nav_msgs::Path();
      }
      else
      {
      }
    }

    if (!goal_reached_)
    { 



      for( unsigned int n = 0; n<path_.poses.size() && n<10000 ;n++)
      {
        // We are tracking.
        // Compute linear velocity.
        v_ = copysign(v_max_, v_);

        //RANDOM NOISE 
        // noisex = (rand() % 100 + -50)/100;
        // noisey = (rand() % 100 + -50)/100;


        //GAUSSIAN NOISE 
        // std::default_random_engine generator;
        // std::normal_distribution<double> distribution(0.0, 0.0125);
        // double noisex = distribution(generator);
        // // std::cout << noisex << std::endl;  
        // std::default_random_engine generator2;
        // double noisey = distribution(generator2);
        // // std::cout << noisex << std::endl;  

        // double w_x1 = path_.poses[n].pose.position.x + noisex;
        // double w_y1 = path_.poses[n].pose.position.y + noisey;


        double w_x1 = path_.poses[n].pose.position.x;
        double w_y1 = path_.poses[n].pose.position.y;
        
        
        // double w_x2 = path_.poses[n+1].pose.position.x;
        // double w_y2 = path_.poses[n+1].pose.position.y;
        double m = 1;
        double q = w_y1 - m*w_x1;
        double a = 1 + m * m;
        double b = 2 * (-x_robot + m*q - m*y_robot);
        double c = q*q + x_robot*x_robot - 2*q*y_robot + y_robot*y_robot - ld_*ld_;


        // double e_x = w_x2 - w_x1; 
        // double e_y = w_y2 - w_y1; 
        // double a = 1 + (e_x*e_x)/(e_y*e_y); 
        // double b = -2*w_x1*((e_x*e_x)/(e_y*e_y))+ 2*w_y1*((e_x*e_y)/(e_y)); 
        // double c = ((w_x1*w_x1)/(e_x*e_x)) * (e_y*e_y) + (w_y1*w_y1) - ((2*w_y1*w_x1*w_y1)/e_x)-ld_*ld_;
        // double m = e_y/e_x;
        // double q = w_y1 - m*w_x1;


        // double a = 1 + m * m;
        // double b = 2 * (-x_robot + m*q - m*y_robot);
        // double c = q*q + x_robot*x_robot - 2*q*y_robot + y_robot*y_robot - ld_*ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = m * x_ld + q;
        
        // Compute the angular velocity y el error lateral
        double yt = sqrt((x_ld-x_robot)*(x_ld-x_robot) + (y_ld-y_robot)*(y_ld-y_robot));
        double ld_2 = ld_ * ld_;
        double vec2goal_x = x_ld-x_robot;
        double vec2goal_y = y_ld-y_robot;

        double alpha = atan2(vec2goal_y, vec2goal_x) - yaw;

        cmd_vel_.angular.z = copysign(std::min( 2*v_ / ld_2 * yt, w_max_ ), sin(alpha));
        
        // Set linear velocity for tracking.
        cmd_vel_.linear.x = v_;

      }
      
    }


    else
    {
      // We are at the goal-> Stop the vehicle
      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::Transform();
      lookahead_.transform.rotation.w = 1.0;
      
      // Stop moving.
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
    }

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);
    
    // Publish the velocities
    pub_vel_.publish(cmd_vel_);
    
  }
  catch (tf2::TransformException &ex)
  {
    std::cout << "error"<< std::endl;  
  }
}

void PurePursuit::receivePath(nav_msgs::Path new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  
  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    idx_ = 0;
    if (new_path.poses.size() > 0)
    {
      goal_reached_ = false;
    }
    else
    {
      goal_reached_ = true;
      std::cout << "error"<< std::endl;  
    }
  }
  else
  {
    std::cout << "error"<< std::endl;  
  }
  
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (robot_footprint) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run()
{
  ros::spin();
}


int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_pursuit");

  PurePursuit controller;
  controller.run();

  return 0;
}
