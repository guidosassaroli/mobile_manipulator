#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "actionlib/server/simple_action_server.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <trajectory_interface/trajectory_interface.h>

#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/hardware_interface_adapter.h>


#include <iostream>

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
        double kp1=0,kp2=50000,kp3=1000,kp4=100,kp5=100,kp6=100;
        double kv1=1,kv2=1,kv3=1,kv4=1,kv5=1,kv6=1;
        double q1,q2,q3,q4,q5,q6;
        double qd1,qd2,qd3,qd4,qd5,qd6;
        double dq[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double q[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double v[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double qr[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double dqr[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double ddqr[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      };           
        double torque[6][1]={{0},
                      {0},
                      {0},
                      {0},
                      {0},
                      {0}
                      }; 
        double kv[6][6] = {
                          {kv1,0,0,0,0,0},
                          {0,kv2,0,0,0,0},
                          {0,0,kv3,0,0,0},
                          {0,0,0,kv4,0,0},
                          {0,0,0,0,kv5,0},
                          {0,0,0,0,0,kv6}
                          };

        double kp[6][6] = { 
                          {kp1,0,0,0,0,0},
                          {0,kp2,0,0,0,0},
                          {0,0,kp3,0,0,0},
                          {0,0,0,kp4,0,0},
                          {0,0,0,0,kp5,0},
                          {0,0,0,0,0,kp6}
                        };

        double gain_[6][1]={{5000},
                      {5000},
                      {100},
                      {100},
                      {100},
                      {100}
                      }; 

        // // void commandCB(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& referencePoint);
        // void commandCB(const std_msgs::Float64ConstPtr& msg)
        // {
        //     command_ = msg->data;
        // }

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

            for (unsigned int i = 0; i < 6; i++){
                qr[i][0] = traj.points[0].positions[i];
                dqr[i][0] = traj.points[0].velocities[i];
                ddqr[i][0] = traj.points[0].accelerations[i];
            }

            // Accept new goal
            gh.setAccepted();            

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

            // has_active_goal_ = false;
            
            // sub_command_ = controller_nh_.subscribe("command",1, &ParComputado::commandCB, this);
            pub_controller_command_ = controller_nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);

            action_server_->start();

            return true;
        }

        //control
        void update(const ros::Time &time, const ros::Duration &duration)
        {
            for(unsigned int i=0;i < 6;i++)
            {
            q[i][0]=joints_[i].getPosition();
            dq[i][0]=joints_[i].getVelocity();
            }
            
            // double aux2=0;
            // int i=0;
            // //for(unsigned int i=0;i < fext.size();i++) fext[i].Zero(); //fuerzas externas a cero (despreciamos)
            // for (aux2=0, i = 0; i < 6; i++){
            //     //for (int j = 0; j < 6; j++){
            //         for (int k = 0; k < 6; k++)
            //         aux2 = kp[i][k]*(qr[k][0]-q[k][0]) + kv[i][k]*(dqr[k][0]-dq[k][0]);
            //     v[i][0]=ddqr[i][0] + aux2;
            // }

            
            // q1=q[0][0];q2=q[1][0];q3=q[2][0];q4=q[3][0];q5=q[4][0];q6=q[5][0];
            // qd1=dq[0][0];qd2=dq[1][0];qd3=dq[2][0];qd4=dq[3][0];qd5=dq[4][0];qd6=dq[5][0];

            // double g = 9.81;

            // double  Ma[6][6] = {
            //  {0.22*sin(q3) + 0.011*sin(q5) - 4.5e-3*cos(q3)*sin(q4) + 7.5e-3*sin(q3)*sin(q5) - 0.41*cos(q2)*cos(q2) - 0.13*cos(q3)*cos(q3) + 2.1e-4*cos(q4)*cos(q4) - 0.22*cos(q2)*cos(q2)*sin(q3) - 0.011*cos(q2)*cos(q2)*sin(q5) - 0.011*cos(q3)*cos(q3)*sin(q5) + 0.27*cos(q2)*cos(q2)*cos(q3)*cos(q3) - 2.1e-4*cos(q2)*cos(q2)*cos(q4)*cos(q4) + 6.6e-4*cos(q2)*cos(q2)*cos(q5)*cos(q5) - 2.1e-4*cos(q3)*cos(q3)*cos(q4)*cos(q4) + 6.6e-4*cos(q3)*cos(q3)*cos(q5)*cos(q5) - 6.6e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 4.5e-3*cos(q2)*cos(q2)*cos(q3)*sin(q4) - 7.5e-3*cos(q2)*cos(q2)*sin(q3)*sin(q5) + 0.021*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q5) - 7.5e-3*cos(q3)*cos(q4)*cos(q5) - 0.22*cos(q2)*cos(q3)*sin(q2) + 4.2e-4*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4) - 1.3e-3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5) + 6.6e-4*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 6.6e-4*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 6.4e-3*cos(q2)*sin(q2)*sin(q4) - 6.4e-3*cos(q3)*sin(q3)*sin(q4) + 7.5e-3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 1.3e-3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 0.013*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q4) + 0.013*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q4) - 0.011*cos(q2)*cos(q4)*cos(q5)*sin(q2) - 0.011*cos(q3)*cos(q4)*cos(q5)*sin(q3) - 0.27*cos(q2)*cos(q3)*sin(q2)*sin(q3) - 7.5e-3*cos(q2)*cos(q3)*sin(q2)*sin(q5) - 4.5e-3*cos(q2)*sin(q2)*sin(q3)*sin(q4) - 4.2e-4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*sin(q2)*sin(q3) + 1.3e-3*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 7.5e-3*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q5) - 1.3e-3*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 0.021*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q5) + 0.021*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 0.021*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q3) + 1.3e-3*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 2.7e-3*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 2.7e-3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 0.49, 3.7e-3*cos(q2)*cos(q5)*sin(q4) - 2.3e-3*cos(q2)*cos(q4) - 3.2e-3*cos(q2)*cos(q4)*sin(q3) - 3.2e-3*cos(q3)*cos(q4)*sin(q2) - 0.082*cos(q2) + 2.1e-4*cos(q2)*cos(q3)*cos(q4)*sin(q4) + 5.3e-3*cos(q2)*cos(q5)*sin(q3)*sin(q4) + 5.3e-3*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 2.1e-4*cos(q4)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 6.6e-4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4), 2.1e-4*cos(q2)*cos(q3)*cos(q4)*sin(q4) - 3.2e-3*cos(q3)*cos(q4)*sin(q2) - 3.2e-3*cos(q2)*cos(q4)*sin(q3) + 5.3e-3*cos(q2)*cos(q5)*sin(q3)*sin(q4) + 5.3e-3*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 2.1e-4*cos(q4)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 6.6e-4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4), 3.6e-4*cos(q2)*sin(q3) + 3.6e-4*cos(q3)*sin(q2) - 2.3e-3*sin(q2)*sin(q4) - 3.2e-3*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*cos(q5)*sin(q3) + 6.6e-4*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 3.2e-3*cos(q2)*cos(q3)*sin(q4) - 3.7e-3*cos(q4)*cos(q5)*sin(q2) + 5.3e-3*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 5.3e-3*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 6.6e-4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5), 2.2e-27*sin(q4)*(3.3e+23*sin(q2)*sin(q3) - 3.3e+23*cos(q2)*cos(q3) + 1.7e+24*sin(q2)*sin(q5) + 2.4e+24*sin(q2)*sin(q3)*sin(q5) - 2.4e+24*cos(q2)*cos(q3)*sin(q5)), 1.6e-6*cos(q2)*sin(q3)*sin(q5) + 1.6e-6*cos(q3)*sin(q2)*sin(q5) - 1.6e-6*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 1.6e-6*cos(q4)*cos(q5)*sin(q2)*sin(q3)},
            //  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 3.7e-3*cos(q2)*cos(q5)*sin(q4) - 2.3e-3*cos(q2)*cos(q4) - 3.2e-3*cos(q2)*cos(q4)*sin(q3) - 3.2e-3*cos(q3)*cos(q4)*sin(q2) - 0.082*cos(q2) + 2.1e-4*cos(q2)*cos(q3)*cos(q4)*sin(q4) + 5.3e-3*cos(q2)*cos(q5)*sin(q3)*sin(q4) + 5.3e-3*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 2.1e-4*cos(q4)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 6.6e-4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4),                                                                                                                                                                                                                                                                                                                             0.22*sin(q3) + 0.011*sin(q5) - 4.5e-3*cos(q3)*sin(q4) + 7.5e-3*sin(q3)*sin(q5) - 2.1e-4*cos(q4)*cos(q4) - 6.6e-4*cos(q5)*cos(q5) + 6.6e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 7.5e-3*cos(q3)*cos(q4)*cos(q5) + 0.41,                                                                                                                                                                                                                                                   0.11*sin(q3) + 0.011*sin(q5) - 2.3e-3*cos(q3)*sin(q4) + 3.7e-3*sin(q3)*sin(q5) - 2.1e-4*cos(q4)*cos(q4) - 6.6e-4*cos(q5)*cos(q5) + 6.6e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 3.7e-3*cos(q3)*cos(q4)*cos(q5) + 0.14,                                                                                                                                                                                                                                                                                                    5.3e-3*cos(q5)*sin(q4) - 2.3e-3*cos(q4)*sin(q3) - 3.2e-3*cos(q4) + 3.7e-3*cos(q5)*sin(q3)*sin(q4) + 6.6e-4*cos(q5)*sin(q4)*sin(q5),                                                                 7.1e-4*cos(q4) - 3.7e-3*cos(q3)*cos(q5) + 5.3e-3*cos(q4)*sin(q5) + 3.7e-3*cos(q4)*sin(q3)*sin(q5),                                                                                                                           -1.6e-6*cos(q5)*sin(q4)},
            //  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           2.1e-4*cos(q2)*cos(q3)*cos(q4)*sin(q4) - 3.2e-3*cos(q3)*cos(q4)*sin(q2) - 3.2e-3*cos(q2)*cos(q4)*sin(q3) + 5.3e-3*cos(q2)*cos(q5)*sin(q3)*sin(q4) + 5.3e-3*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 2.1e-4*cos(q4)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 6.6e-4*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4),                                                                                                                                                                                                                                                                                                                             0.11*sin(q3) + 0.011*sin(q5) - 2.3e-3*cos(q3)*sin(q4) + 3.7e-3*sin(q3)*sin(q5) - 2.1e-4*cos(q4)*cos(q4) - 6.6e-4*cos(q5)*cos(q5) + 6.6e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 3.7e-3*cos(q3)*cos(q4)*cos(q5) + 0.14,                                                                                                                                                                                                                                                                                                                                                                        0.011*sin(q5) - 4.5e-4*sin(q4)*sin(q4) + 6.6e-4*sin(q4)*sin(q4)*sin(q5)*sin(q5) + 0.14,                                                                                                                                                                                                                                                                                                                                                              5.3e-3*cos(q5)*sin(q4) - 3.2e-3*cos(q4) + 6.6e-4*cos(q5)*sin(q4)*sin(q5),                                                                                                                       2.2e-27*cos(q4)*(2.4e+24*sin(q5) + 3.3e+23),                                                                                                                           -1.6e-6*cos(q5)*sin(q4)},
            //  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 3.6e-4*cos(q2)*sin(q3) + 3.6e-4*cos(q3)*sin(q2) - 2.3e-3*sin(q2)*sin(q4) - 3.2e-3*sin(q2)*sin(q3)*sin(q4) + 6.6e-4*cos(q2)*cos(q5)*cos(q5)*sin(q3) + 6.6e-4*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 3.2e-3*cos(q2)*cos(q3)*sin(q4) - 3.7e-3*cos(q4)*cos(q5)*sin(q2) + 5.3e-3*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 5.3e-3*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 6.6e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 6.6e-4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                    5.3e-3*cos(q5)*sin(q4) - 2.3e-3*cos(q4)*sin(q3) - 3.2e-3*cos(q4) + 3.7e-3*cos(q5)*sin(q3)*sin(q4) + 6.6e-4*cos(q5)*sin(q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                    5.3e-3*cos(q5)*sin(q4) - 3.2e-3*cos(q4) + 6.6e-4*cos(q5)*sin(q4)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                             6.6e-4*cos(q5)*cos(q5) + 3.6e-4,                                                                                                                                                                 0,                                                                                                                                    1.6e-6*sin(q5)},
            //  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     2.2e-27*sin(q4)*(3.3e+23*sin(q2)*sin(q3) - 3.3e+23*cos(q2)*cos(q3) + 1.7e+24*sin(q2)*sin(q5) + 2.4e+24*sin(q2)*sin(q3)*sin(q5) - 2.4e+24*cos(q2)*cos(q3)*sin(q5)),                                                                                                                                                                                                                                                                                                                                                                                                                     7.1e-4*cos(q4) - 3.7e-3*cos(q3)*cos(q5) + 5.3e-3*cos(q4)*sin(q5) + 3.7e-3*cos(q4)*sin(q3)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                 2.2e-27*cos(q4)*(2.4e+24*sin(q5) + 3.3e+23),                                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                            7.1e-4,                                                                                                                                                 0},
            //  {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      1.6e-6*cos(q2)*sin(q3)*sin(q5) + 1.6e-6*cos(q3)*sin(q2)*sin(q5) - 1.6e-6*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 1.6e-6*cos(q4)*cos(q5)*sin(q2)*sin(q3),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               -1.6e-6*cos(q5)*sin(q4),                                                                                                                                                                                                                                                                                                                                                                                                                     -1.6e-6*cos(q5)*sin(q4),                                                                                                                                                                                                                                                                                                                                                                                                                        1.6e-6*sin(q5),                                                                                                                                                                 0,                                                                                                                                            1.6e-6}
            //  };   

            // double aux = 1.3e-3*qd4*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) + 5.3e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 5.3e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 5.3e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 5.3e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 0.015*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 7.5e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.021*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd2*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) + 2.7e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 2.7e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 2.7e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 2.7e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 2.7e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 2.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 0.025*qd1*qd2*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4) - 0.025*qd1*qd3*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4) + 7.5e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 7.5e-3*qd1*qd5*cos(q2)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 1.3e-3*qd2*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.021*qd1*qd4*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 0.021*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4) - 0.021*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 0.021*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 1.3e-3*qd2*qd3*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd2*qd3*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 0.042*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 0.042*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd2*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) + 1.3e-3*qd3*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) + 8.5e-4*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*sin(q2)*sin(q3)*sin(q4) - 2.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 1.3e-3*qd2*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 1.3e-3*qd3*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 2.7e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 2.7e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 5.3e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 5.3e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 2.7e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 2.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
            // double Va[6][6] =  {
            // {0.2*qd1 + 0.082*qd2*qd2*sin(q2) + 0.22*qd1*qd2*cos(q3) + 0.22*qd1*qd3*cos(q3) + 0.011*qd1*qd5*cos(q5) + 6.4e-3*qd1*qd2*sin(q4) + 6.4e-3*qd1*qd3*sin(q4) + 0.41*qd1*qd2*sin(2.0*q2) + 0.13*qd1*qd2*sin(2.0*q3) + 0.13*qd1*qd3*sin(2.0*q2) + 0.13*qd1*qd3*sin(2.0*q3) - 2.1e-4*qd1*qd4*sin(2.0*q4) + 2.3e-3*qd2*qd2*cos(q4)*sin(q2) - 2.3e-3*qd4*qd4*cos(q4)*sin(q2) + 4.5e-3*qd1*qd2*sin(q3)*sin(q4) + 4.5e-3*qd1*qd3*sin(q3)*sin(q4) - 1.5e-4*qd2*qd4*sin(q2)*sin(q3) - 1.5e-4*qd3*qd4*sin(q2)*sin(q3) - 0.43*qd1*qd2*cos(q2)*cos(q2)*cos(q3) - 0.22*qd1*qd3*cos(q2)*cos(q2)*cos(q3) - 0.011*qd1*qd5*cos(q2)*cos(q2)*cos(q5) - 0.011*qd1*qd5*cos(q3)*cos(q3)*cos(q5) - 0.013*qd1*qd2*cos(q2)*cos(q2)*sin(q4) - 0.013*qd1*qd2*cos(q3)*cos(q3)*sin(q4) - 0.013*qd1*qd3*cos(q2)*cos(q2)*sin(q4) - 0.013*qd1*qd3*cos(q3)*cos(q3)*sin(q4) - 3.2e-3*qd2*qd2*cos(q2)*cos(q3)*cos(q4) - 3.2e-3*qd3*qd3*cos(q2)*cos(q3)*cos(q4) + 3.2e-3*qd4*qd4*cos(q2)*cos(q3)*cos(q4) + 3.2e-3*qd2*qd2*cos(q4)*sin(q2)*sin(q3) + 3.2e-3*qd3*qd3*cos(q4)*sin(q2)*sin(q3) - 3.7e-3*qd2*qd2*cos(q5)*sin(q2)*sin(q4) - 3.2e-3*qd4*qd4*cos(q4)*sin(q2)*sin(q3) + 3.7e-3*qd4*qd4*cos(q5)*sin(q2)*sin(q4) + 3.7e-3*qd5*qd5*cos(q5)*sin(q2)*sin(q4) + 1.5e-4*qd2*qd4*cos(q2)*cos(q3) + 0.011*qd1*qd2*cos(q4)*cos(q5) - 4.5e-3*qd1*qd4*cos(q3)*cos(q4) + 1.5e-4*qd3*qd4*cos(q2)*cos(q3) + 0.011*qd1*qd3*cos(q4)*cos(q5) + 7.5e-3*qd1*qd2*cos(q3)*sin(q5) + 7.5e-3*qd1*qd3*cos(q3)*sin(q5) + 7.5e-3*qd1*qd5*cos(q5)*sin(q3) - 0.021*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 4.5e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q4) - 0.021*qd1*qd2*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 0.021*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 4.2e-4*qd2*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4) - 0.021*qd1*qd3*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 1.3e-3*qd2*qd4*cos(q2)*cos(q3)*cos(q5)*cos(q5) + 4.2e-4*qd3*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4) + 1.3e-3*qd3*qd4*cos(q2)*cos(q3)*cos(q5)*cos(q5) - 0.54*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*sin(q2) + 4.2e-4*qd1*qd2*cos(q2)*cos(q4)*cos(q4)*sin(q2) - 0.54*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q3) - 0.54*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*sin(q2) - 1.3e-3*qd1*qd2*cos(q2)*cos(q5)*cos(q5)*sin(q2) + 4.2e-4*qd1*qd3*cos(q2)*cos(q4)*cos(q4)*sin(q2) - 0.54*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q3) + 4.2e-4*qd1*qd2*cos(q3)*cos(q4)*cos(q4)*sin(q3) - 0.015*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q5) - 1.3e-3*qd1*qd3*cos(q2)*cos(q5)*cos(q5)*sin(q2) - 1.3e-3*qd1*qd2*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 4.2e-4*qd1*qd3*cos(q3)*cos(q4)*cos(q4)*sin(q3) - 7.5e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q5) - 1.3e-3*qd1*qd3*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 4.2e-4*qd1*qd4*cos(q2)*cos(q2)*cos(q4)*sin(q4) + 4.2e-4*qd1*qd4*cos(q3)*cos(q3)*cos(q4)*sin(q4) - 7.5e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q5)*sin(q3) + 1.3e-3*qd1*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd5*cos(q3)*cos(q3)*cos(q5)*sin(q5) + 1.3e-3*qd1*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 9.0e-3*qd1*qd2*cos(q2)*cos(q2)*sin(q3)*sin(q4) - 4.5e-3*qd1*qd3*cos(q2)*cos(q2)*sin(q3)*sin(q4) - 4.2e-4*qd2*qd4*cos(q4)*cos(q4)*sin(q2)*sin(q3) - 1.3e-3*qd2*qd4*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 4.2e-4*qd3*qd4*cos(q4)*cos(q4)*sin(q2)*sin(q3) - 1.3e-3*qd3*qd4*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 0.021*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5) + 0.025*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q4) + 0.025*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q4) + 5.3e-3*qd2*qd2*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 5.3e-3*qd3*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 5.3e-3*qd4*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 5.3e-3*qd5*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 2.1e-4*qd2*qd2*cos(q2)*cos(q4)*sin(q3)*sin(q4) - 2.1e-4*qd2*qd2*cos(q3)*cos(q4)*sin(q2)*sin(q4) - 2.1e-4*qd3*qd3*cos(q2)*cos(q4)*sin(q3)*sin(q4) - 2.1e-4*qd3*qd3*cos(q3)*cos(q4)*sin(q2)*sin(q4) - 5.3e-3*qd2*qd2*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 5.3e-3*qd3*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 5.3e-3*qd4*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 5.3e-3*qd5*qd5*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 6.4e-3*qd2*qd3*cos(q2)*cos(q3)*cos(q4) - 1.4e-3*qd4*qd5*cos(q2)*cos(q3)*cos(q4) - 6.4e-3*qd1*qd4*cos(q2)*cos(q4)*sin(q2) + 1.3e-3*qd1*qd5*cos(q2)*cos(q4)*sin(q2) + 7.5e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q3) - 6.4e-3*qd1*qd4*cos(q3)*cos(q4)*sin(q3) + 7.5e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q3) + 1.3e-3*qd1*qd5*cos(q3)*cos(q4)*sin(q3) + 1.3e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q5) + 7.5e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q4) + 1.3e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q5) + 7.5e-3*qd1*qd5*cos(q3)*cos(q4)*sin(q5) + 1.6e-6*qd2*qd6*cos(q2)*cos(q3)*sin(q5) + 1.6e-6*qd3*qd6*cos(q2)*cos(q3)*sin(q5) + 1.6e-6*qd5*qd6*cos(q2)*cos(q5)*sin(q3) + 1.6e-6*qd5*qd6*cos(q3)*cos(q5)*sin(q2) + 0.43*qd1*qd2*cos(q2)*sin(q2)*sin(q3) + 0.22*qd1*qd3*cos(q2)*sin(q2)*sin(q3) + 0.021*qd1*qd2*cos(q2)*sin(q2)*sin(q5) + 0.021*qd1*qd3*cos(q2)*sin(q2)*sin(q5) + 0.021*qd1*qd2*cos(q3)*sin(q3)*sin(q5) + 6.4e-3*qd2*qd3*cos(q4)*sin(q2)*sin(q3) + 0.021*qd1*qd3*cos(q3)*sin(q3)*sin(q5) + 5.0e-5*qd2*qd5*cos(q2)*sin(q3)*sin(q4) + 5.0e-5*qd2*qd5*cos(q3)*sin(q2)*sin(q4) + 5.0e-5*qd3*qd5*cos(q2)*sin(q3)*sin(q4) + 5.0e-5*qd3*qd5*cos(q3)*sin(q2)*sin(q4) + 1.4e-3*qd4*qd5*cos(q4)*sin(q2)*sin(q3) + 7.5e-3*qd4*qd5*cos(q4)*sin(q2)*sin(q5) - 1.6e-6*qd2*qd6*sin(q2)*sin(q3)*sin(q5) - 1.6e-6*qd3*qd6*sin(q2)*sin(q3)*sin(q5) + 0.013*qd1*qd4*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) - 0.015*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.013*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) - 2.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) - 7.5e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 2.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) - 2.7e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) - 7.5e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 2.7e-3*qd1*qd5*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 2.7e-3*qd1*qd2*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 2.7e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) - 7.5e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2.7e-3*qd1*qd3*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 2.7e-3*qd1*qd5*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 0.042*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) - 0.042*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) - 0.042*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) - 0.042*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) + 1.3e-3*qd2*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd2*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 1.3e-3*qd3*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd3*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 1.3e-3*qd4*qd5*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 0.042*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.042*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 1.3e-3*qd2*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 1.3e-3*qd3*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 8.5e-4*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*sin(q2) + 2.7e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 8.5e-4*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*sin(q3) - 8.5e-4*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*sin(q2) - 1.3e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 2.7e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 2.7e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 8.5e-4*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*sin(q3) - 1.3e-3*qd1*qd3*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 2.7e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) - 1.3e-3*qd1*qd2*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 8.5e-4*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q4) - 1.3e-3*qd1*qd3*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 1.3e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd1*qd4*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 2.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd5*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 1.3e-3*qd2*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd3*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 6.6e-4*qd2*qd2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) + 6.6e-4*qd3*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) - 6.6e-4*qd4*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) - 6.6e-4*qd2*qd2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 6.6e-4*qd3*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 6.6e-4*qd4*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 7.5e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2) + 0.011*qd2*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 1.6e-6*qd2*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.6e-6*qd2*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.6e-6*qd3*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.6e-6*qd3*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.011*qd4*qd5*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 1.6e-6*qd4*qd6*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 1.6e-6*qd5*qd6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 9.0e-3*qd1*qd2*cos(q2)*cos(q3)*sin(q2)*sin(q4) - 4.5e-3*qd1*qd3*cos(q2)*cos(q3)*sin(q2)*sin(q4) - 4.5e-3*qd1*qd4*cos(q2)*cos(q4)*sin(q2)*sin(q3) + 0.011*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q4) - 4.2e-4*qd2*qd3*cos(q2)*cos(q4)*sin(q3)*sin(q4) - 4.2e-4*qd2*qd3*cos(q3)*cos(q4)*sin(q2)*sin(q4) + 0.011*qd1*qd5*cos(q2)*cos(q4)*sin(q2)*sin(q5) + 0.011*qd1*qd4*cos(q3)*cos(q5)*sin(q3)*sin(q4) + 0.011*qd1*qd5*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 1.3e-3*qd4*qd5*cos(q2)*cos(q5)*sin(q3)*sin(q5) - 1.3e-3*qd4*qd5*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 0.015*qd1*qd2*cos(q2)*sin(q2)*sin(q3)*sin(q5) + 7.5e-3*qd1*qd3*cos(q2)*sin(q2)*sin(q3)*sin(q5) - 0.011*qd2*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 0.011*qd4*qd5*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 1.6e-6*qd4*qd6*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 1.6e-6*qd5*qd6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 6.6e-4*qd2*qd2*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 6.6e-4*qd2*qd2*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 6.6e-4*qd3*qd3*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 6.6e-4*qd3*qd3*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + aux},
            // {0.2*qd2 - 0.11*qd1*qd1*cos(q3) + 0.11*qd3*qd3*cos(q3) - 3.2e-3*qd1*qd1*sin(q4) + 3.2e-3*qd4*qd4*sin(q4) - 0.21*qd1*qd1*sin(2.0*q2) - 0.067*qd1*qd1*sin(2.0*q3) + 0.22*qd2*qd3*cos(q3) + 0.011*qd2*qd5*cos(q5) + 0.011*qd3*qd5*cos(q5) - 1.4e-3*qd4*qd5*sin(q4) + 0.22*qd1*qd1*cos(q2)*cos(q2)*cos(q3) + 6.4e-3*qd1*qd1*cos(q2)*cos(q2)*sin(q4) + 6.4e-3*qd1*qd1*cos(q3)*cos(q3)*sin(q4) + 2.1e-4*qd2*qd4*sin(2.0*q4) + 2.1e-4*qd3*qd4*sin(2.0*q4) + 6.6e-4*qd2*qd5*sin(2.0*q5) + 6.6e-4*qd3*qd5*sin(2.0*q5) - 5.3e-3*qd1*qd1*cos(q4)*cos(q5) + 5.3e-3*qd4*qd4*cos(q4)*cos(q5) + 5.3e-3*qd5*qd5*cos(q4)*cos(q5) - 3.7e-3*qd1*qd1*cos(q3)*sin(q5) + 3.7e-3*qd3*qd3*cos(q3)*sin(q5) + 3.7e-3*qd5*qd5*cos(q3)*sin(q5) - 2.3e-3*qd1*qd1*sin(q3)*sin(q4) + 2.3e-3*qd3*qd3*sin(q3)*sin(q4) + 2.3e-3*qd4*qd4*sin(q3)*sin(q4) + 5.7e-4*qd1*qd4*sin(q2)*sin(q3) + 4.5e-3*qd2*qd3*sin(q3)*sin(q4) - 0.011*qd4*qd5*sin(q4)*sin(q5) + 1.6e-6*qd5*qd6*sin(q4)*sin(q5) + 0.011*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 0.011*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.27*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2) - 2.1e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*sin(q2) + 0.27*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3) + 6.6e-4*qd1*qd1*cos(q2)*cos(q5)*cos(q5)*sin(q2) - 2.1e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*sin(q3) + 7.5e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q5) + 6.6e-4*qd1*qd1*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 4.5e-3*qd1*qd1*cos(q2)*cos(q2)*sin(q3)*sin(q4) + 1.3e-3*qd4*qd5*cos(q5)*cos(q5)*sin(q4) - 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q4) - 3.7e-3*qd1*qd1*cos(q4)*cos(q5)*sin(q3) - 6.6e-4*qd1*qd1*cos(q4)*cos(q5)*sin(q5) + 3.7e-3*qd3*qd3*cos(q4)*cos(q5)*sin(q3) + 3.7e-3*qd4*qd4*cos(q4)*cos(q5)*sin(q3) + 3.7e-3*qd5*qd5*cos(q4)*cos(q5)*sin(q3) + 6.6e-4*qd4*qd4*cos(q4)*cos(q5)*sin(q5) - 0.22*qd1*qd1*cos(q2)*sin(q2)*sin(q3) - 0.011*qd1*qd1*cos(q2)*sin(q2)*sin(q5) - 0.011*qd1*qd1*cos(q3)*sin(q3)*sin(q5) - 5.7e-4*qd1*qd4*cos(q2)*cos(q3) - 4.5e-3*qd2*qd4*cos(q3)*cos(q4) - 4.5e-3*qd3*qd4*cos(q3)*cos(q4) - 1.6e-6*qd4*qd6*cos(q4)*cos(q5) + 4.5e-3*qd1*qd4*cos(q2)*sin(q4) + 7.5e-3*qd2*qd3*cos(q3)*sin(q5) + 7.5e-3*qd2*qd5*cos(q5)*sin(q3) + 7.5e-3*qd3*qd5*cos(q5)*sin(q3) + 0.021*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) + 0.021*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) + 4.2e-4*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4) - 1.3e-3*qd2*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd3*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd2*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd3*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 4.2e-4*qd1*qd4*cos(q4)*cos(q4)*sin(q2)*sin(q3) - 0.021*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 4.2e-4*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 4.2e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*sin(q3) + 6.6e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 6.6e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 4.5e-3*qd1*qd1*cos(q2)*cos(q3)*sin(q2)*sin(q4) - 7.5e-3*qd1*qd1*cos(q2)*sin(q2)*sin(q3)*sin(q5) + 7.5e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5) - 1.6e-6*qd1*qd6*cos(q2)*cos(q3)*sin(q5) + 7.5e-3*qd2*qd3*cos(q4)*cos(q5)*sin(q3) + 7.5e-3*qd2*qd4*cos(q3)*cos(q5)*sin(q4) + 7.5e-3*qd2*qd5*cos(q3)*cos(q4)*sin(q5) + 7.5e-3*qd3*qd4*cos(q3)*cos(q5)*sin(q4) + 7.5e-3*qd3*qd5*cos(q3)*cos(q4)*sin(q5) + 6.4e-3*qd1*qd4*cos(q2)*sin(q3)*sin(q4) + 6.4e-3*qd1*qd4*cos(q3)*sin(q2)*sin(q4) - 1.4e-3*qd1*qd5*cos(q2)*sin(q3)*sin(q4) - 1.4e-3*qd1*qd5*cos(q3)*sin(q2)*sin(q4) - 7.5e-3*qd1*qd5*cos(q2)*sin(q4)*sin(q5) + 1.6e-6*qd1*qd6*sin(q2)*sin(q3)*sin(q5) - 7.5e-3*qd4*qd5*sin(q3)*sin(q4)*sin(q5) + 7.5e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) + 1.3e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 1.3e-3*qd1*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd1*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 2.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 7.5e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.3e-3*qd1*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.013*qd1*qd1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4) + 0.011*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.011*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 1.6e-6*qd1*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 1.6e-6*qd1*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.011*qd1*qd5*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.011*qd1*qd5*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.021*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 2.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.3e-3*qd1*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)},
            // {0.2*qd3 - 0.11*qd1*qd1*cos(q3) - 0.11*qd2*qd2*cos(q3) - 3.2e-3*qd1*qd1*sin(q4) + 3.2e-3*qd4*qd4*sin(q4) - 0.067*qd1*qd1*sin(2.0*q2) - 0.067*qd1*qd1*sin(2.0*q3) + 0.011*qd2*qd5*cos(q5) + 0.011*qd3*qd5*cos(q5) - 1.4e-3*qd4*qd5*sin(q4) + 0.11*qd1*qd1*cos(q2)*cos(q2)*cos(q3) + 6.4e-3*qd1*qd1*cos(q2)*cos(q2)*sin(q4) + 6.4e-3*qd1*qd1*cos(q3)*cos(q3)*sin(q4) + 2.1e-4*qd2*qd4*sin(2.0*q4) + 2.1e-4*qd3*qd4*sin(2.0*q4) + 6.6e-4*qd2*qd5*sin(2.0*q5) + 6.6e-4*qd3*qd5*sin(2.0*q5) - 5.3e-3*qd1*qd1*cos(q4)*cos(q5) + 5.3e-3*qd4*qd4*cos(q4)*cos(q5) + 5.3e-3*qd5*qd5*cos(q4)*cos(q5) - 3.7e-3*qd1*qd1*cos(q3)*sin(q5) - 3.7e-3*qd2*qd2*cos(q3)*sin(q5) - 2.3e-3*qd1*qd1*sin(q3)*sin(q4) - 2.3e-3*qd2*qd2*sin(q3)*sin(q4) + 5.7e-4*qd1*qd4*sin(q2)*sin(q3) - 0.011*qd4*qd5*sin(q4)*sin(q5) + 1.6e-6*qd5*qd6*sin(q4)*sin(q5) + 0.011*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 0.011*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.27*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2) - 2.1e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*sin(q2) + 0.27*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3) + 6.6e-4*qd1*qd1*cos(q2)*cos(q5)*cos(q5)*sin(q2) - 2.1e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*sin(q3) + 3.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q5) + 6.6e-4*qd1*qd1*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 2.3e-3*qd1*qd1*cos(q2)*cos(q2)*sin(q3)*sin(q4) + 1.3e-3*qd4*qd5*cos(q5)*cos(q5)*sin(q4) - 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q4) - 3.7e-3*qd1*qd1*cos(q4)*cos(q5)*sin(q3) - 3.7e-3*qd2*qd2*cos(q4)*cos(q5)*sin(q3) - 6.6e-4*qd1*qd1*cos(q4)*cos(q5)*sin(q5) + 6.6e-4*qd4*qd4*cos(q4)*cos(q5)*sin(q5) - 0.11*qd1*qd1*cos(q2)*sin(q2)*sin(q3) - 0.011*qd1*qd1*cos(q2)*sin(q2)*sin(q5) - 0.011*qd1*qd1*cos(q3)*sin(q3)*sin(q5) - 5.7e-4*qd1*qd4*cos(q2)*cos(q3) - 1.6e-6*qd4*qd6*cos(q4)*cos(q5) + 0.021*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) + 0.021*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) + 4.2e-4*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4) - 1.3e-3*qd2*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd3*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd2*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd3*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 4.2e-4*qd1*qd4*cos(q4)*cos(q4)*sin(q2)*sin(q3) - 0.021*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 4.2e-4*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 4.2e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*sin(q3) + 6.6e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 6.6e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 2.3e-3*qd1*qd1*cos(q2)*cos(q3)*sin(q2)*sin(q4) - 3.7e-3*qd1*qd1*cos(q2)*sin(q2)*sin(q3)*sin(q5) - 1.6e-6*qd1*qd6*cos(q2)*cos(q3)*sin(q5) + 6.4e-3*qd1*qd4*cos(q2)*sin(q3)*sin(q4) + 6.4e-3*qd1*qd4*cos(q3)*sin(q2)*sin(q4) - 1.4e-3*qd1*qd5*cos(q2)*sin(q3)*sin(q4) - 1.4e-3*qd1*qd5*cos(q3)*sin(q2)*sin(q4) + 1.6e-6*qd1*qd6*sin(q2)*sin(q3)*sin(q5) + 3.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) + 1.3e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 1.3e-3*qd1*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd1*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 2.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 3.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.3e-3*qd1*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.013*qd1*qd1*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q4) + 0.011*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.011*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 1.6e-6*qd1*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 1.6e-6*qd1*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.011*qd1*qd5*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.011*qd1*qd5*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.021*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 2.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.3e-3*qd1*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)},
            // {0.2*qd4 + 1.1e-4*qd1*qd1*sin(2.0*q4) - 1.1e-4*qd2*qd2*sin(2.0*q4) - 1.1e-4*qd3*qd3*sin(2.0*q4) + 1.6e-6*qd5*qd6*cos(q5) + 5.0e-5*qd2*qd5*sin(q4) + 5.0e-5*qd3*qd5*sin(q4) - 2.1e-4*qd2*qd3*sin(2.0*q4) - 6.6e-4*qd4*qd5*sin(2.0*q5) + 2.3e-3*qd1*qd1*cos(q3)*cos(q4) + 2.3e-3*qd2*qd2*cos(q3)*cos(q4) - 5.7e-4*qd1*qd2*sin(q2)*sin(q3) - 5.7e-4*qd1*qd3*sin(q2)*sin(q3) - 2.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4) - 2.1e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*sin(q4) - 2.1e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*sin(q4) - 6.6e-4*qd1*qd1*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 6.6e-4*qd2*qd2*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 6.6e-4*qd3*qd3*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 1.3e-3*qd2*qd5*cos(q5)*cos(q5)*sin(q4) + 1.3e-3*qd3*qd5*cos(q5)*cos(q5)*sin(q4) + 3.2e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2) + 3.2e-3*qd1*qd1*cos(q3)*cos(q4)*sin(q3) - 3.7e-3*qd1*qd1*cos(q3)*cos(q5)*sin(q4) - 3.7e-3*qd2*qd2*cos(q3)*cos(q5)*sin(q4) + 5.7e-4*qd1*qd2*cos(q2)*cos(q3) + 5.7e-4*qd1*qd3*cos(q2)*cos(q3) + 1.6e-6*qd2*qd6*cos(q4)*cos(q5) + 1.6e-6*qd3*qd6*cos(q4)*cos(q5) - 4.5e-3*qd1*qd2*cos(q2)*sin(q4) - 4.2e-4*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q4) - 4.2e-4*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q4) + 1.3e-3*qd2*qd3*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 4.2e-4*qd1*qd2*cos(q4)*cos(q4)*sin(q2)*sin(q3) + 4.2e-4*qd1*qd3*cos(q4)*cos(q4)*sin(q2)*sin(q3) + 4.2e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q4) + 6.6e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 6.6e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 2.3e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2)*sin(q3) - 5.3e-3*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q4) - 5.3e-3*qd1*qd1*cos(q3)*cos(q5)*sin(q3)*sin(q4) - 7.5e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q5) + 5.0e-5*qd1*qd5*cos(q2)*cos(q3)*cos(q4) - 6.4e-3*qd1*qd2*cos(q2)*sin(q3)*sin(q4) - 6.4e-3*qd1*qd2*cos(q3)*sin(q2)*sin(q4) - 6.4e-3*qd1*qd3*cos(q2)*sin(q3)*sin(q4) - 6.4e-3*qd1*qd3*cos(q3)*sin(q2)*sin(q4) - 5.0e-5*qd1*qd5*cos(q4)*sin(q2)*sin(q3) - 6.4e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) - 6.4e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) + 3.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 1.3e-3*qd1*qd5*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 1.3e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 1.3e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 1.3e-3*qd1*qd2*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*qd1*qd3*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 3.7e-3*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 6.6e-4*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 6.6e-4*qd1*qd1*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 0.011*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 0.011*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.011*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 0.011*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 1.6e-6*qd1*qd6*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 1.3e-3*qd1*qd5*cos(q2)*cos(q5)*sin(q3)*sin(q5) - 1.3e-3*qd1*qd5*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 1.6e-6*qd1*qd6*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 0.011*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + 0.011*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4) + 1.3e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) - 4.2e-4*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*sin(q2)*sin(q3)*sin(q4) - 1.3e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 1.3e-3*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) - 1.3e-3*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 1.3e-3*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4)},
            // {0.2*qd5 - 5.3e-3*qd1*qd1*cos(q5) - 5.3e-3*qd2*qd2*cos(q5) - 5.3e-3*qd3*qd3*cos(q5) - 3.3e-4*qd2*qd2*sin(2.0*q5) - 3.3e-4*qd3*qd3*sin(2.0*q5) + 3.3e-4*qd4*qd4*sin(2.0*q5) - 0.011*qd2*qd3*cos(q5) - 1.6e-6*qd4*qd6*cos(q5) - 5.0e-5*qd2*qd4*sin(q4) - 5.0e-5*qd3*qd4*sin(q4) + 5.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q5) + 5.3e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q5) - 6.6e-4*qd2*qd3*sin(2.0*q5) - 3.7e-3*qd1*qd1*cos(q5)*sin(q3) - 3.7e-3*qd2*qd2*cos(q5)*sin(q3) - 1.6e-6*qd2*qd6*sin(q4)*sin(q5) - 1.6e-6*qd3*qd6*sin(q4)*sin(q5) + 3.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q5)*sin(q3) + 6.6e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q5)*sin(q5) + 6.6e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q5)*sin(q5) - 6.6e-4*qd1*qd1*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 6.6e-4*qd2*qd2*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 6.6e-4*qd3*qd3*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd2*qd4*cos(q5)*cos(q5)*sin(q4) - 1.3e-3*qd3*qd4*cos(q5)*cos(q5)*sin(q4) - 0.011*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5) - 6.6e-4*qd1*qd1*cos(q2)*cos(q4)*sin(q2) - 6.6e-4*qd1*qd1*cos(q3)*cos(q4)*sin(q3) - 3.7e-3*qd1*qd1*cos(q3)*cos(q4)*sin(q5) - 3.7e-3*qd2*qd2*cos(q3)*cos(q4)*sin(q5) + 1.3e-3*qd2*qd3*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q5) + 6.6e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 6.6e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 3.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2) - 5.3e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 5.3e-3*qd1*qd1*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 5.0e-5*qd1*qd4*cos(q2)*cos(q3)*cos(q4) - 1.6e-6*qd1*qd6*cos(q2)*cos(q5)*sin(q3) - 1.6e-6*qd1*qd6*cos(q3)*cos(q5)*sin(q2) + 1.4e-3*qd1*qd2*cos(q2)*sin(q3)*sin(q4) + 1.4e-3*qd1*qd2*cos(q3)*sin(q2)*sin(q4) + 1.4e-3*qd1*qd3*cos(q2)*sin(q3)*sin(q4) + 1.4e-3*qd1*qd3*cos(q3)*sin(q2)*sin(q4) + 7.5e-3*qd1*qd2*cos(q2)*sin(q4)*sin(q5) + 5.0e-5*qd1*qd4*cos(q4)*sin(q2)*sin(q3) + 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) + 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) + 1.3e-3*qd1*qd1*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 3.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 1.3e-3*qd1*qd1*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 1.3e-3*qd1*qd2*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) - 1.3e-3*qd1*qd2*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 1.3e-3*qd1*qd3*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) - 1.3e-3*qd1*qd3*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 1.3e-3*qd1*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 2.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 2.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.011*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3) - 1.3e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 3.7e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 1.6e-6*qd1*qd6*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 1.3e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q3)*sin(q5) + 1.3e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 0.011*qd1*qd2*cos(q2)*sin(q3)*sin(q4)*sin(q5) + 0.011*qd1*qd2*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.011*qd1*qd3*cos(q2)*sin(q3)*sin(q4)*sin(q5) + 0.011*qd1*qd3*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 1.6e-6*qd1*qd6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 0.011*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 0.011*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 1.3e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) + 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 1.3e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.3e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 1.3e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5)},
            // {0.2*qd6 + 1.6e-6*qd4*qd5*cos(q5) + 1.6e-6*qd2*qd5*sin(q4)*sin(q5) + 1.6e-6*qd3*qd5*sin(q4)*sin(q5) - 1.6e-6*qd2*qd4*cos(q4)*cos(q5) - 1.6e-6*qd3*qd4*cos(q4)*cos(q5) + 1.6e-6*qd1*qd2*cos(q2)*cos(q3)*sin(q5) + 1.6e-6*qd1*qd3*cos(q2)*cos(q3)*sin(q5) + 1.6e-6*qd1*qd5*cos(q2)*cos(q5)*sin(q3) + 1.6e-6*qd1*qd5*cos(q3)*cos(q5)*sin(q2) - 1.6e-6*qd1*qd2*sin(q2)*sin(q3)*sin(q5) - 1.6e-6*qd1*qd3*sin(q2)*sin(q3)*sin(q5) + 1.6e-6*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.6e-6*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.6e-6*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.6e-6*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.6e-6*qd1*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 1.6e-6*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 1.6e-6*qd1*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 1.6e-6*qd1*qd5*cos(q4)*sin(q2)*sin(q3)*sin(q5)}
            // };
            // double Ga[6][6] = {
            // {                                                                                                                                                                                                                                                                    0},
            // {g*(0.94*cos(q2)*cos(q3) - 2.4*sin(q2) - 0.94*sin(q2)*sin(q3) - 0.033*sin(q2)*sin(q3)*sin(q5) + 0.033*cos(q2)*cos(q3)*sin(q5) + 0.02*cos(q2)*sin(q3)*sin(q4) + 0.02*cos(q3)*sin(q2)*sin(q4) + 0.033*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.033*cos(q3)*cos(q4)*cos(q5)*sin(q2))},
            // {             g*(0.94*cos(q2)*cos(q3) - 0.94*sin(q2)*sin(q3) - 0.033*sin(q2)*sin(q3)*sin(q5) + 0.033*cos(q2)*cos(q3)*sin(q5) + 0.02*cos(q2)*sin(q3)*sin(q4) + 0.02*cos(q3)*sin(q2)*sin(q4) + 0.033*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.033*cos(q3)*cos(q4)*cos(q5)*sin(q2))},
            // {                                                                                                                                                                                                             -3.5e-4*g*cos(q2 + q3)*(56.0*cos(q4) - 93.0*cos(q5)*sin(q4))},
            // {                                                                                                                        g*(0.033*cos(q2)*cos(q5)*sin(q3) + 0.033*cos(q3)*cos(q5)*sin(q2) + 0.033*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 0.033*cos(q4)*sin(q2)*sin(q3)*sin(q5))},
            // {                                                                                                                                                                                                                                                                       0}
            // };

            // double aux3=0;
            // for (aux3=0, i = 0; i < 6; i++){
            //     //for (int j = 0; j < 6; j++){
            //         for (int k = 0; k < 6; k++)
            //         aux3 = Ma[i][k]*v[k][0];
            //     torque[i][0]=Va[i][0] + Ga[i][0] + aux3;
            // }


            //torque = Ma *. v.data + Va + Ga;
            for(unsigned int i=0;i < joints_.size();i++){
            double error = qr[i][0] - joints_[i].getPosition();
            joints_[i].setCommand(error*gain_[i][0]);
            // joints_[i].setCommand(torque[i][0]);
            }
           
        }

        void starting(const ros::Time &time) {

            for(unsigned int i=0; i<6; i++){
                q[i][0]=joints_[i].getPosition();  //posicion actual del robot (real)
                dq[i][0]=joints_[i].getVelocity();
            }

            for (unsigned int i = 0; i < 6; i++){
                dqr[i][0] = 0;
                ddqr[i][0] = 0;
            }

            // qr[0][0] = 0;
            // qr[1][0] = -1.57;
            // qr[2][0] = 2.625;
            // qr[3][0] = 0;
            // qr[4][0] = -1,57;
            // qr[5][0] = 0;   

            qr[0][0] = 0;
            qr[1][0] = 0;
            qr[2][0] = 0;
            qr[3][0] = 0;
            qr[4][0] = 0;
            qr[5][0] = 0;           
            
            struct sched_param param;
            param.sched_priority=sched_get_priority_max(SCHED_FIFO);
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
            double Wn = 1;
            double Xi = 1;
            trajectory_msgs::JointTrajectory traj;
            ros::Publisher pub_controller_command_;
            ros::Subscriber sub_command_;
            std::string name_;               ///< Controller name.
            //actionlib::

            // typedef actionlib::ActionServer<control_msgs::JointTrajectoryAction> as;
            // typedef as::GoalHandle GoalHandle;

            std::vector<hardware_interface::JointHandle> joints_;

            bool has_active_goal_;
            GoalHandle active_goal_;

            std::vector<std::string> joint_names_;
            
    };
    PLUGINLIB_EXPORT_CLASS(par_computado_ns::ParComputado, controller_interface::ControllerBase);
}