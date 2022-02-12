#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>

#include "actionlib/server/simple_action_server.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>

#include <cmath>
#include <iostream>
#include <ctime>

const double DEFAULT_GOAL_THRESHOLD = 0.1;

namespace par_computado_ns
{

    class ParComputado : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>                  ActionServer;
        typedef boost::shared_ptr<ActionServer>                                                     ActionServerPtr;
        typedef ActionServer::GoalHandle                                                            GoalHandle;

        ActionServerPtr    action_server_;


        //definicion parametros control
        double dq[6][1] = {{0},{0},{0},{0},{0},{0}}; 
        double q[6][1]={{0},{0},{0},{0},{0},{0}}; 
        double v[6][1]={{0},{0},{0},{0},{0},{0}}; 
        double qr[6][1]={{0},{0},{0},{0},{0},{0}}; 
        double dqr[6][1]={{0},{0},{0},{0},{0},{0}}; 
        double ddqr[6][1]={{0},{0},{0},{0},{0},{0}};           
        double torque[6][1]={{0},{0},{0},{0},{0},{0}}; 


        double gain_[6][1]={{4000},
                      {8000},
                      {4000},
                      {10},
                      {10},
                      {10}
                      }; 
        
        double gain_int[6][1]={{10},
                      {10},
                      {10},
                      {10},
                      {10},
                      {10}
                      }; 

        int n;
        double error_integral;
        double error_ant = 0, U, error = 0;
        double t, T, tant;

        //goalCB
        void goalCB(GoalHandle gh)
        {
            // Cancels the currently active goal.
            if (has_active_goal_)
            {
                // Stops the controller.
                trajectory_msgs::JointTrajectory empty;
                empty.joint_names = joint_names_;
                pub_controller_command_.publish(empty);

                // Marks the current goal as canceled.
                active_goal_.setCanceled();
                has_active_goal_ = false;
            }

            gh.setAccepted();
            active_goal_ = gh;
            has_active_goal_ = true;

            // Sends the trajectory along to the controller
            traj = gh.getGoal()->trajectory;
            pub_controller_command_.publish(traj);

            n = traj.points.size();            
            for (unsigned int i = 0; i < 6; i++){                
                qr[i][0] = traj.points[n-1].positions[i];
                dqr[i][0] = traj.points[n-1].velocities[i];
                ddqr[i][0] = traj.points[n-1].accelerations[i];
            }

        }

        void cancelCB(GoalHandle gh)
        {
            if (active_goal_ == gh)
            {
            // Stops the controller.
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_controller_command_.publish(empty);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
            }
        }
        

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n, ros::NodeHandle& controller_nh_)
        {
            //get the joint that we want to control

            XmlRpc::XmlRpcValue joint_names;
            if(!n.getParam("arm_controller/joints",joint_names))
            {
            ROS_ERROR("No ’joints’ in controller. (namespace: %s)", n.getNamespace().c_str());
            return false;
            }
            if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
            ROS_ERROR("’joints’ is not a struct. (namespace: %s)", n.getNamespace().c_str());
            return false;
            }
            for(int i=0; i < joint_names.size();i++)
            {
            XmlRpc::XmlRpcValue &name_value=joint_names[i];
            if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
            ROS_ERROR("joints are not strings. (namespace: %s)", n.getNamespace().c_str());
            return false;
            }
            joint_=hw->getHandle((std::string)name_value);
            joints_.push_back(joint_);
            }

            // ROS API: Action interface
            action_server_.reset(new ActionServer(controller_nh_, "/arm_controller/follow_joint_trajectory",
                                                    boost::bind(&ParComputado::goalCB,   this, _1),
                                                    boost::bind(&ParComputado::cancelCB, this, _1),
                                                    false));

            pub_controller_command_ = controller_nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
            pub_graficas1 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q1_dq1", 1);
            pub_graficas2 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q2_dq2", 1);
            pub_graficas3 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q3_dq3", 1);
            pub_graficas4 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q4_dq4", 1);
            pub_graficas5 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q5_dq5", 1);
            pub_graficas6 = controller_nh_.advertise<geometry_msgs::PoseStamped>("q6_dq6", 1);

            action_server_->start();

            return true;
        }

        //control
        void update(const ros::Time &time, const ros::Duration &duration)
        {
            for(unsigned int i = 0; i < 6; i++){
            q[i][0] = joints_[i].getPosition();
            dq[i][0] = joints_[i].getVelocity();
            }

            t = clock();
            T = (double(t-tant)/CLOCKS_PER_SEC);

            for(unsigned int i=0; i < joints_.size();i++){
                error = qr[i][0] - joints_[i].getPosition();
                if ((error_ant < 0 && error > 0) || (error_ant > 0 && error < 0)){ 
                    error_integral = 0;
                    std::cout << "PI Controller" << std::endl;
                }
                error_integral = error_integral + error*T;
                U = error*gain_[i][0] + error_integral*gain_int[i][0];

                joints_[i].setCommand(copysign(std::min(abs(U), 50.0), U));

            }
            error_ant = error;
            tant = t;


            current_time = ros::Time::now();

            msg1.header.stamp = current_time;
            msg1.pose.position.x = q[0][0];
            msg1.pose.position.y = qr[0][0];
            msg1.pose.position.z = dq[0][0];
            msg1.pose.orientation.x = dqr[0][0];
            msg1.pose.orientation.y = torque[0][0];
            pub_graficas1.publish(msg1);

            msg2.header.stamp = current_time;
            msg2.pose.position.x = q[1][0];
            msg2.pose.position.y = qr[1][0];
            msg2.pose.position.z = dq[1][0];
            msg2.pose.orientation.x = dqr[1][0];
            msg2.pose.orientation.y = torque[1][0];
            pub_graficas2.publish(msg2);

            msg3.header.stamp = current_time;
            msg3.pose.position.x = q[2][0];
            msg3.pose.position.y = qr[2][0];
            msg3.pose.position.z = dq[2][0];
            msg3.pose.orientation.x = dqr[2][0];
            msg3.pose.orientation.y = torque[2][0];
            pub_graficas3.publish(msg3);

            msg4.header.stamp = current_time;
            msg4.pose.position.x = q[3][0];
            msg4.pose.position.y = qr[3][0];
            msg4.pose.position.z = dq[3][0];
            msg4.pose.orientation.x = dqr[3][0];
            msg4.pose.orientation.y = torque[3][0];
            pub_graficas4.publish(msg4);

            msg5.header.stamp = current_time;
            msg5.pose.position.x = q[4][0];
            msg5.pose.position.y = qr[4][0];
            msg5.pose.position.z = dq[4][0];
            msg5.pose.orientation.x = dqr[4][0];
            msg5.pose.orientation.y = torque[4][0];
            pub_graficas5.publish(msg5);

            msg6.header.stamp = current_time;
            msg6.pose.position.x = q[5][0];
            msg6.pose.position.y = qr[5][0];
            msg6.pose.position.z = dq[5][0];
            msg6.pose.orientation.x = dqr[5][0];
            msg6.pose.orientation.y = torque[5][0];
            pub_graficas6.publish(msg6);
        }

        void starting(const ros::Time &time) {

            for(unsigned int i=0; i<6; i++){
                q[i][0] = joints_[i].getPosition();  // posicion actual del robot (real)
                dq[i][0] = joints_[i].getVelocity();
            }

            for (unsigned int i = 0; i < 6; i++){
                dqr[i][0] = 0;
                ddqr[i][0] = 0;
            }

            qr[0][0] = 0;
            qr[1][0] = 0;
            qr[2][0] = 0;
            qr[3][0] = 0;
            qr[4][0] = 0;
            qr[5][0] = 0;

            tant = clock();
                       
            struct sched_param param;
            param.sched_priority = sched_get_priority_max(SCHED_FIFO);
            if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
            {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
            }

        }
        void stopping(const ros::Time &time) {}

        private:
            hardware_interface::JointHandle joint_;
            double command_;
            trajectory_msgs::JointTrajectory traj;
            ros::Publisher pub_controller_command_;
            ros::Subscriber sub_command_;
            std::string name_;               ///< Controller name.

            std::vector<hardware_interface::JointHandle> joints_;

            bool has_active_goal_;
            GoalHandle active_goal_;

            std::vector<std::string> joint_names_;

            ros::Publisher pub_graficas1, pub_graficas2, pub_graficas3, pub_graficas4, pub_graficas5, pub_graficas6;
            geometry_msgs::PoseStamped msg1, msg2, msg3, msg4, msg5, msg6;
            ros::Time current_time;
            
    };
    PLUGINLIB_EXPORT_CLASS(par_computado_ns::ParComputado, controller_interface::ControllerBase);
}