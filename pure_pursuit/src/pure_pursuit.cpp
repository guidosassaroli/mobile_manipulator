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

using std::string;

class PurePursuit
{
public:

  PurePursuit();

  //It calculates the velocity 
  void computeVelocity(nav_msgs::Odometry odom);

  //It receive the path from move_base
  void getPath(nav_msgs::Path path);

  //It calculate de coordinate of the goal with respect to the robot
  KDL::Frame transformPoint(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf);
  
  //Function used to calculate distances in the x-y plane.

  double distance(double x1, double y1, double x2, double y2)
  {   
    return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2));
  }
  

  //Run the controller
  void run();
  
private:

  // Position tolerace and lookahead distance
  double ld_, pos_tol_;
  // Max Velocity, linear velocity, max rotational velocity 
  double v_max_, vel, w_max_;

  nav_msgs::Path path_;
  unsigned ind_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_;
  //A Class which provides coordinate transforms between any two frames in a system
  tf2_ros::Buffer tf_buffer_;
  //this class automatically subscribes to ROS transform messages
  tf2_ros::TransformListener tf_listener_;
  //This class provides an easy way to publish coordinate frame transform information
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_;
  
};

PurePursuit::PurePursuit() : ld_(1.0), v_max_(0.5), vel(v_max_), w_max_(2.5), pos_tol_(0.1), ind_(0),
                             goal_reached_(true), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("map"), robot_frame_id_("robot_footprint"),
                             lookahead_frame_id_("lookahead")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("lookahead_distance", ld_, 0.2);
  nh_private_.param<double>("linear_velocity", vel, 0.5);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 2.5);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.2);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
  // Frame attached to midpoint of rear axle .
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "robot_footprint");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");

  //Message with lookahaed
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;
  
  // /move_base/TrajectoryPlannerROS/local_plan
  sub_path_ = nh_.subscribe("/move_base/TrajectoryPlannerROS/global_plan", 1, &PurePursuit::getPath, this);
  sub_odom_ = nh_.subscribe("/odometry/filtered", 1, &PurePursuit::computeVelocity, this);
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/ommp_velocity_controller/cmd_vel", 1);

}

void PurePursuit::computeVelocity(nav_msgs::Odometry odom)
{ 
  // The velocity commands are computed, each time a new Odometry message is received.
  // We use the odometry/filtered thorugh the tf tree

  // Get the current robot pose
  geometry_msgs::TransformStamped tf;

  tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
  // We compute the point to follow, based on our current pose, path information and lookahead distance.
  for (; ind_ < path_.poses.size(); ind_++)
  {
    if (distance(path_.poses[ind_].pose.position.x, path_.poses[ind_].pose.position.y, tf.transform.translation.x, tf.transform.translation.y) >= ld_)
    {

      // We consider the lateral error as the transformation between the lookahaed frame and the robot_footprint frame
      KDL::Frame error = transformPoint(path_.poses[ind_].pose, tf.transform);
      lookahead_.transform.translation.x = error.p.x();
      lookahead_.transform.translation.y = error.p.y();
      lookahead_.transform.translation.z = error.p.z();
      error.M.GetQuaternion(lookahead_.transform.rotation.x,
                              lookahead_.transform.rotation.y,
                              lookahead_.transform.rotation.z,
                              lookahead_.transform.rotation.w);

      break;
    }
  }



  if (!path_.poses.empty() && ind_ >= path_.poses.size())
  {
    // We are approaching the goal, which is closer than ld
    // This is the pose of the goal w.r.t. the robot_footprint frame
    KDL::Frame goal_rf = transformPoint(path_.poses.back().pose, tf.transform);

    if (fabs(goal_rf.p.x()) <= pos_tol_)
    {
      // We have reached the goal
      goal_reached_ = true;
      // Reset the path
      path_ = nav_msgs::Path();
    }
    else
    {
      // We need to extend the lookahead distance beyond the goal point.
      // this is an approximation: we find the intersection between the circle of radius ld
      // centered at the robot (origin) and the line defined by the last path pose
      double roll, pitch, yaw;
      // .M = orientation of the frame
      goal_rf.M.GetRPY(roll, pitch, yaw);
      double m = tan(yaw); // Slope of line defined by the last path pose
      // intersection entre y = q + m*x y x^2 + y^2 = ld^2
      double q = goal_rf.p.y() - m * goal_rf.p.x();
      double a = 1 + m * m;
      double b = 2 * q * m;
      double c = q * q - ld_ * ld_;
      double D = sqrt(b * b - 4 * a * c);
      double x_ld = (-b + copysign(D, vel)) / (2 * a);
      double y_ld = m * x_ld + q;

      lookahead_.transform.translation.x = x_ld;
      lookahead_.transform.translation.y = y_ld;
      lookahead_.transform.translation.z = goal_rf.p.z();
      goal_rf.M.GetQuaternion(lookahead_.transform.rotation.x,
                               lookahead_.transform.rotation.y,
                               lookahead_.transform.rotation.z,
                               lookahead_.transform.rotation.w);
    }
  }

  if (!goal_reached_)
  {

    // - Determine the currente locationof the vehicle.
    // - Find the path point closest to the vehicle.
    // - Find the goal point
    // - Transform the goal point to vehicle coordinates (with the function TransformPoint)
    // - Calculate the curvature and request the vehicle to set the steering to that curvature.
    // - Update the vehicle’s position
    // - Compute linear velocity: we take magnitude of v_max and the sign of y.
    vel = copysign(v_max_, vel);

    // Compute the angular velocity.
    // Lateral error is the y-value of the lookahead point (in robot_footprint frame)
    double gamma = lookahead_.transform.translation.y;
    double ld_pow2 = ld_ * ld_;
    cmd_vel_.angular.z = std::min(2 * gamma * vel / ld_pow2 , w_max_);

    // Set linear velocity for tracking.
    cmd_vel_.linear.x = vel;
  }
  else
  {
    // Goal reached
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

void PurePursuit::getPath(nav_msgs::Path new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.
  
  if (new_path.header.frame_id == map_frame_id_)
  {
    path_ = new_path;
    ind_ = 0;
    if (new_path.poses.size() > 0)
    {
      goal_reached_ = false;
    }
    else
    {
      goal_reached_ = true;
    std::cout << "The path is empty." << std::endl;
    }
  }
  else
  {
    std::cout << "The path is not published correctly." << std::endl;
  }
  
}

KDL::Frame PurePursuit::transformPoint(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame T_goal_map(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (robot_footprint) in global (map) frame
  KDL::Frame T_robot_map(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  return T_robot_map.Inverse()*T_goal_map;
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
