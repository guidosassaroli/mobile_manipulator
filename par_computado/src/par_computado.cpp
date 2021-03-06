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
        double kp1 = 1750, kp2 = 1750, kp3 = 1750, kp4 = 500, kp5 = 750, kp6 = 1;
        double kv1 = 50, kv2 = 50, kv3 = 50, kv4 = 15, kv5 = 25, kv6 = 1;
        double q1, q2, q3, q4, q5, q6;
        double qd1, qd2, qd3, qd4, qd5, qd6;
        double dq[6][1] = {{0},{0},{0},{0},{0},{0}};
        double q[6][1] = {{0},{0},{0},{0},{0},{0}};
        double dq_[6][1] = {{0},{0},{0},{0},{0},{0}};
        double q_[6][1] = {{0},{0},{0},{0},{0},{0}};
        double v[6][1] = {{0},{0},{0},{0},{0},{0}};
        double qr[6][1] = {{0},{0},{0},{0},{0},{0}};
        double dqr[6][1] = {{0},{0},{0},{0},{0},{0}};
        double ddqr[6][1] = {{0},{0},{0},{0},{0},{0}};       
        double torque[6][1] = {{0},{0},{0},{0},{0},{0}};
        double qr_[6][1] = {{0},{0},{0},{0},{0},{0}};
        double dqr_[6][1] = {{0},{0},{0},{0},{0},{0}};
        double ddqr_[6][1] = {{0},{0},{0},{0},{0},{0}};       
        double torque_[6][1] = {{0},{0},{0},{0},{0},{0}};
        
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

        bool action_torque = false;
        int contador = 0;
        int n;
        double error_integral;
        double error_ant = 0, U, error = 0;
        double t, T, tant;
        bool aux_6 = false;
        double torque_ant;


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

            action_torque = true;

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

            qr_[0][0] = qr[2][0];   // shoulder pan
            dqr_[0][0] = dqr[2][0];
            ddqr_[0][0] = ddqr[2][0];

            qr_[1][0] = qr[1][0];   // shoulder lift
            dqr_[1][0] = dqr[1][0];
            ddqr_[1][0] = ddqr[1][0];

            qr_[2][0] = qr[0][0];   // elbow
            dqr_[2][0] = dqr[0][0];
            ddqr_[2][0] = ddqr[0][0];

            for (unsigned int i = 3; i < 6; i++){                
                qr_[i][0] = qr[i][0];
                dqr_[i][0] = dqr[i][0];
                ddqr_[i][0] = ddqr[i][0];
            }

            aux_6 = false;
        }

        void cancelCB(GoalHandle gh)
        {
            if (active_goal_ == gh){
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
            ROS_ERROR("No ???joints??? in controller. (namespace: %s)", n.getNamespace().c_str());
            return false;
            }
            if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
            ROS_ERROR("???joints??? is not a struct. (namespace: %s)", n.getNamespace().c_str());
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

            q_[0][0] = q[2][0];   // shoulder pan
            dq_[0][0] = dq[2][0];

            q_[1][0] = q[1][0];   // shoulder lift
            dq_[1][0] = dq[1][0];

            q_[2][0] = q[0][0];   // elbow
            dq_[2][0] = dq[0][0];

            for (unsigned int i = 3; i < 6; i++){                
                q_[i][0] = q[i][0];
                dq_[i][0] = dq[i][0];
            }
        
            double aux2 = 0;
            for (unsigned int i = 0; i < 6; i++){
                for (int k = 0; k < 6; k++){
                    aux2 = aux2 + kp[i][k]*(qr_[k][0]-q_[k][0]) + kv[i][k]*(dqr_[k][0]-dq_[k][0]);
                    }
                v[i][0] = ddqr_[i][0] + aux2;
                aux2 = 0;
            }
            
            q1 = q_[0][0];
            q2 = q_[1][0] - 1.57;       // shoulder lift
            q3 = q_[2][0] + 1.57;       // elbow
            q4 = q_[3][0];
            q5 = q_[4][0] + 3.14;       // wrist 2
            q6 = q_[5][0];

            qd1 = dq_[0][0];
            qd2 = dq_[1][0];
            qd3 = dq_[2][0];
            qd4 = dq_[3][0];
            qd5 = dq_[4][0];
            qd6 = dq_[5][0];

            double g = 9.81;

            double Ma[6][6] =  {
                                {0.22*sin(q3) + 0.013*sin(q5) + 9.2e-3*sin(q3)*sin(q5) - 0.3*cos(q2)*cos(q2) - 0.14*cos(q3)*cos(q3) - 0.22*cos(q2)*cos(q2)*sin(q3) - 0.013*cos(q2)*cos(q2)*sin(q5) - 0.013*cos(q3)*cos(q3)*sin(q5) + 0.27*cos(q2)*cos(q2)*cos(q3)*cos(q3) + 8.4e-4*cos(q2)*cos(q2)*cos(q5)*cos(q5) + 8.4e-4*cos(q3)*cos(q3)*cos(q5)*cos(q5) - 8.4e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 9.2e-3*cos(q2)*cos(q2)*sin(q3)*sin(q5) + 0.026*cos(q2)*cos(q2)*cos(q3)*cos(q3)*sin(q5) - 9.2e-3*cos(q3)*cos(q4)*cos(q5) - 0.22*cos(q2)*cos(q3)*sin(q2) - 1.7e-3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5) + 8.4e-4*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 8.4e-4*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 9.2e-3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 1.7e-3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 0.013*cos(q2)*cos(q4)*cos(q5)*sin(q2) - 0.013*cos(q3)*cos(q4)*cos(q5)*sin(q3) - 0.27*cos(q2)*cos(q3)*sin(q2)*sin(q3) - 9.2e-3*cos(q2)*cos(q3)*sin(q2)*sin(q5) + 1.7e-3*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 9.2e-3*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q5) - 1.7e-3*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 0.026*cos(q2)*cos(q3)*sin(q2)*sin(q3)*sin(q5) + 0.026*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 0.026*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q3) + 1.7e-3*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 3.4e-3*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 3.4e-3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 0.46, 1.7e-27*cos(q5)*sin(q4)*(2.7e+24*cos(q2) + 3.7e+24*cos(q2)*sin(q3) + 3.7e+24*cos(q3)*sin(q2) + 4.8e+23*cos(q2)*sin(q3)*sin(q5) + 4.8e+23*cos(q3)*sin(q2)*sin(q5) - 4.8e+23*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.8e+23*cos(q4)*cos(q5)*sin(q2)*sin(q3)), 1.7e-27*cos(q5)*sin(q4)*(3.7e+24*cos(q2)*sin(q3) + 3.7e+24*cos(q3)*sin(q2) + 4.8e+23*cos(q2)*sin(q3)*sin(q5) + 4.8e+23*cos(q3)*sin(q2)*sin(q5) - 4.8e+23*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.8e+23*cos(q4)*cos(q5)*sin(q2)*sin(q3)), 1.5e-4*cos(q2)*sin(q3) + 1.5e-4*cos(q3)*sin(q2) + 8.4e-4*cos(q2)*cos(q5)*cos(q5)*sin(q3) + 8.4e-4*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 4.6e-3*cos(q4)*cos(q5)*sin(q2) + 6.5e-3*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 6.5e-3*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 8.4e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 8.4e-4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5), 3.5e-27*sin(q4)*(2.6e+23*sin(q2)*sin(q3) - 2.6e+23*cos(q2)*cos(q3) + 1.3e+24*sin(q2)*sin(q5) + 1.9e+24*sin(q2)*sin(q3)*sin(q5) - 1.9e+24*cos(q2)*cos(q3)*sin(q5)), 4.0e-6*cos(q2)*sin(q3)*sin(q5) + 4.0e-6*cos(q3)*sin(q2)*sin(q5) - 4.0e-6*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.0e-6*cos(q4)*cos(q5)*sin(q2)*sin(q3)},
                                {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            1.7e-27*cos(q5)*sin(q4)*(2.7e+24*cos(q2) + 3.7e+24*cos(q2)*sin(q3) + 3.7e+24*cos(q3)*sin(q2) + 4.8e+23*cos(q2)*sin(q3)*sin(q5) + 4.8e+23*cos(q3)*sin(q2)*sin(q5) - 4.8e+23*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.8e+23*cos(q4)*cos(q5)*sin(q2)*sin(q3)),                                                                                                         0.22*sin(q3) + 0.013*sin(q5) + 9.2e-3*sin(q3)*sin(q5) - 8.4e-4*cos(q5)*cos(q5) + 8.4e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 9.2e-3*cos(q3)*cos(q4)*cos(q5) + 0.81,                                                                                       0.11*sin(q3) + 0.013*sin(q5) + 4.6e-3*sin(q3)*sin(q5) - 8.4e-4*cos(q5)*cos(q5) + 8.4e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 4.6e-3*cos(q3)*cos(q4)*cos(q5) + 0.14,                                                                                                                                                                                                                                                                      1.7e-27*cos(q5)*sin(q4)*(2.7e+24*sin(q3) + 4.8e+23*sin(q5) + 3.7e+24),                                                                 8.9e-4*cos(q4) - 4.6e-3*cos(q3)*cos(q5) + 6.5e-3*cos(q4)*sin(q5) + 4.6e-3*cos(q4)*sin(q3)*sin(q5),                                                                                                                           -4.0e-6*cos(q5)*sin(q4)},
                                {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              1.7e-27*cos(q5)*sin(q4)*(3.7e+24*cos(q2)*sin(q3) + 3.7e+24*cos(q3)*sin(q2) + 4.8e+23*cos(q2)*sin(q3)*sin(q5) + 4.8e+23*cos(q3)*sin(q2)*sin(q5) - 4.8e+23*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.8e+23*cos(q4)*cos(q5)*sin(q2)*sin(q3)),                                                                                                         0.11*sin(q3) + 0.013*sin(q5) + 4.6e-3*sin(q3)*sin(q5) - 8.4e-4*cos(q5)*cos(q5) + 8.4e-4*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 4.6e-3*cos(q3)*cos(q4)*cos(q5) + 0.14,                                                                                                                                                                0.013*sin(q5) - 8.4e-4*sin(q4)*sin(q4) + 8.4e-4*sin(q4)*sin(q4)*sin(q5)*sin(q5) + 0.21,                                                                                                                                                                                                                                                                                        1.7e-27*cos(q5)*sin(q4)*(4.8e+23*sin(q5) + 3.7e+24),                                                                                                                       3.5e-27*cos(q4)*(1.9e+24*sin(q5) + 2.6e+23),                                                                                                                           -4.0e-6*cos(q5)*sin(q4)},
                                {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       1.5e-4*cos(q2)*sin(q3) + 1.5e-4*cos(q3)*sin(q2) + 8.4e-4*cos(q2)*cos(q5)*cos(q5)*sin(q3) + 8.4e-4*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 4.6e-3*cos(q4)*cos(q5)*sin(q2) + 6.5e-3*cos(q2)*cos(q3)*cos(q4)*cos(q5) - 6.5e-3*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 8.4e-4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 8.4e-4*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5),                                                                                                                                                                                 1.7e-27*cos(q5)*sin(q4)*(2.7e+24*sin(q3) + 4.8e+23*sin(q5) + 3.7e+24),                                                                                                                                                                                 1.7e-27*cos(q5)*sin(q4)*(4.8e+23*sin(q5) + 3.7e+24),                                                                                                                                                                                                                                                                                                                   8.4e-4*cos(q5)*cos(q5) + 0.039,                                                                                                                                                                 0,                                                                                                                                    4.0e-6*sin(q5)},
                                {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                3.5e-27*sin(q4)*(2.6e+23*sin(q2)*sin(q3) - 2.6e+23*cos(q2)*cos(q3) + 1.3e+24*sin(q2)*sin(q5) + 1.9e+24*sin(q2)*sin(q3)*sin(q5) - 1.9e+24*cos(q2)*cos(q3)*sin(q5)),                                                                                                                                                     8.9e-4*cos(q4) - 4.6e-3*cos(q3)*cos(q5) + 6.5e-3*cos(q4)*sin(q5) + 4.6e-3*cos(q4)*sin(q3)*sin(q5),                                                                                                                                                                                         3.5e-27*cos(q4)*(1.9e+24*sin(q5) + 2.6e+23),                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                              0.02,                                                                                                                                                 0},
                                {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                4.0e-6*cos(q2)*sin(q3)*sin(q5) + 4.0e-6*cos(q3)*sin(q2)*sin(q5) - 4.0e-6*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 4.0e-6*cos(q4)*cos(q5)*sin(q2)*sin(q3),                                                                                                                                                                                                                               -4.0e-6*cos(q5)*sin(q4),                                                                                                                                                                                                             -4.0e-6*cos(q5)*sin(q4),                                                                                                                                                                                                                                                                                                                             4.0e-6*sin(q5),                                                                                                                                                                 0,                                                                                                                                            1.4e-3}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                };

            double aux = - 3.4e-3*qd1*qd3*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 3.4e-3*qd1*qd5*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 0.052*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) - 0.052*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) - 0.052*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) - 0.052*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) + 1.7e-3*qd2*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd2*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 1.7e-3*qd3*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd3*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 1.7e-3*qd4*qd5*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 0.052*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.052*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 1.7e-3*qd2*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 1.7e-3*qd3*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 3.4e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 3.4e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 3.4e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd3*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 3.4e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) - 1.7e-3*qd1*qd2*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 1.7e-3*qd1*qd3*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 1.7e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd1*qd4*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 3.4e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd5*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 1.7e-3*qd2*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd3*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 8.4e-4*qd2*qd2*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) + 8.4e-4*qd3*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) - 8.4e-4*qd4*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) - 8.4e-4*qd2*qd2*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 8.4e-4*qd3*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 8.4e-4*qd4*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 9.2e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2) + 0.013*qd2*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 4.0e-6*qd2*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 4.0e-6*qd2*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 4.0e-6*qd3*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 4.0e-6*qd3*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.013*qd4*qd5*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 4.0e-6*qd4*qd6*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 4.0e-6*qd5*qd6*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 0.013*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q4) + 0.013*qd1*qd5*cos(q2)*cos(q4)*sin(q2)*sin(q5) + 0.013*qd1*qd4*cos(q3)*cos(q5)*sin(q3)*sin(q4) + 0.013*qd1*qd5*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 1.7e-3*qd4*qd5*cos(q2)*cos(q5)*sin(q3)*sin(q5) - 1.7e-3*qd4*qd5*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 0.018*qd1*qd2*cos(q2)*sin(q2)*sin(q3)*sin(q5) + 9.2e-3*qd1*qd3*cos(q2)*sin(q2)*sin(q3)*sin(q5) - 0.013*qd2*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 0.013*qd4*qd5*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 4.0e-6*qd4*qd6*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 4.0e-6*qd5*qd6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 8.4e-4*qd2*qd2*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 8.4e-4*qd2*qd2*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 8.4e-4*qd3*qd3*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 8.4e-4*qd3*qd3*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 1.7e-3*qd4*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) + 6.7e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 6.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 6.7e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 6.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 0.018*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 9.2e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.026*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd2*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5) + 3.4e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 3.4e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 3.4e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 3.4e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 3.4e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 3.4e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 9.2e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 9.2e-3*qd1*qd5*cos(q2)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q2)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 1.7e-3*qd2*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 0.026*qd1*qd4*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4) - 0.026*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4) - 0.026*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 0.026*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) + 1.7e-3*qd2*qd3*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd2*qd3*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 0.052*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 0.052*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd2*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) + 1.7e-3*qd3*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 3.4e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 1.7e-3*qd2*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 1.7e-3*qd3*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 3.4e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 3.4e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 6.7e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 6.7e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 3.4e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 3.4e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5);
            double Va[6][6] =  {
                                {0.2*qd1 + 0.22*qd1*qd2*cos(q3) + 0.22*qd1*qd3*cos(q3) + 0.013*qd1*qd5*cos(q5) + 0.3*qd1*qd2*sin(2.0*q2) + 0.14*qd1*qd2*sin(2.0*q3) + 0.14*qd1*qd3*sin(2.0*q2) + 0.14*qd1*qd3*sin(2.0*q3) - 1.5e-4*qd2*qd4*sin(q2)*sin(q3) - 1.5e-4*qd3*qd4*sin(q2)*sin(q3) - 0.45*qd1*qd2*cos(q2)*cos(q2)*cos(q3) - 0.22*qd1*qd3*cos(q2)*cos(q2)*cos(q3) - 0.013*qd1*qd5*cos(q2)*cos(q2)*cos(q5) - 0.013*qd1*qd5*cos(q3)*cos(q3)*cos(q5) - 4.6e-3*qd2*qd2*cos(q5)*sin(q2)*sin(q4) + 4.6e-3*qd4*qd4*cos(q5)*sin(q2)*sin(q4) + 4.6e-3*qd5*qd5*cos(q5)*sin(q2)*sin(q4) + 1.5e-4*qd2*qd4*cos(q2)*cos(q3) + 0.013*qd1*qd2*cos(q4)*cos(q5) + 1.5e-4*qd3*qd4*cos(q2)*cos(q3) + 0.013*qd1*qd3*cos(q4)*cos(q5) + 9.2e-3*qd1*qd2*cos(q3)*sin(q5) + 9.2e-3*qd1*qd3*cos(q3)*sin(q5) + 9.2e-3*qd1*qd5*cos(q5)*sin(q3) - 0.026*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5) - 0.026*qd1*qd2*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 0.026*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5) - 0.026*qd1*qd3*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 1.7e-3*qd2*qd4*cos(q2)*cos(q3)*cos(q5)*cos(q5) + 1.7e-3*qd3*qd4*cos(q2)*cos(q3)*cos(q5)*cos(q5) - 0.55*qd1*qd2*cos(q2)*cos(q3)*cos(q3)*sin(q2) - 0.55*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q3) - 0.55*qd1*qd3*cos(q2)*cos(q3)*cos(q3)*sin(q2) - 1.7e-3*qd1*qd2*cos(q2)*cos(q5)*cos(q5)*sin(q2) - 0.55*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q3) - 0.018*qd1*qd2*cos(q2)*cos(q2)*cos(q3)*sin(q5) - 1.7e-3*qd1*qd3*cos(q2)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd2*cos(q3)*cos(q5)*cos(q5)*sin(q3) - 9.2e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q3)*sin(q5) - 1.7e-3*qd1*qd3*cos(q3)*cos(q5)*cos(q5)*sin(q3) - 9.2e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q5)*sin(q3) + 1.7e-3*qd1*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd5*cos(q3)*cos(q3)*cos(q5)*sin(q5) + 1.7e-3*qd1*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd2*qd4*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd3*qd4*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 0.026*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5) + 6.5e-3*qd2*qd2*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 6.5e-3*qd3*qd3*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 6.5e-3*qd4*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 6.5e-3*qd5*qd5*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 6.5e-3*qd2*qd2*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 6.5e-3*qd3*qd3*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.5e-3*qd4*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 6.5e-3*qd5*qd5*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 1.7e-3*qd4*qd5*cos(q2)*cos(q3)*cos(q4) + 1.7e-3*qd1*qd5*cos(q2)*cos(q4)*sin(q2) + 9.2e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q3) + 9.2e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q3) + 1.7e-3*qd1*qd5*cos(q3)*cos(q4)*sin(q3) + 1.7e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q5) + 9.2e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q4) + 1.7e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q5) + 9.2e-3*qd1*qd5*cos(q3)*cos(q4)*sin(q5) + 4.0e-6*qd2*qd6*cos(q2)*cos(q3)*sin(q5) + 4.0e-6*qd3*qd6*cos(q2)*cos(q3)*sin(q5) + 4.0e-6*qd5*qd6*cos(q2)*cos(q5)*sin(q3) + 4.0e-6*qd5*qd6*cos(q3)*cos(q5)*sin(q2) + 0.45*qd1*qd2*cos(q2)*sin(q2)*sin(q3) + 0.22*qd1*qd3*cos(q2)*sin(q2)*sin(q3) + 0.026*qd1*qd2*cos(q2)*sin(q2)*sin(q5) + 0.026*qd1*qd3*cos(q2)*sin(q2)*sin(q5) + 0.026*qd1*qd2*cos(q3)*sin(q3)*sin(q5) + 0.026*qd1*qd3*cos(q3)*sin(q3)*sin(q5) + 5.2e-5*qd2*qd5*cos(q2)*sin(q3)*sin(q4) + 5.2e-5*qd2*qd5*cos(q3)*sin(q2)*sin(q4) + 5.2e-5*qd3*qd5*cos(q2)*sin(q3)*sin(q4) + 5.2e-5*qd3*qd5*cos(q3)*sin(q2)*sin(q4) + 1.7e-3*qd4*qd5*cos(q4)*sin(q2)*sin(q3) + 9.2e-3*qd4*qd5*cos(q4)*sin(q2)*sin(q5) - 4.0e-6*qd2*qd6*sin(q2)*sin(q3)*sin(q5) - 4.0e-6*qd3*qd6*sin(q2)*sin(q3)*sin(q5) - 0.018*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 3.4e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) - 9.2e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 3.4e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) - 3.4e-3*qd1*qd2*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) - 9.2e-3*qd1*qd4*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 3.4e-3*qd1*qd5*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 3.4e-3*qd1*qd2*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 3.4e-3*qd1*qd3*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) - 9.2e-3*qd1*qd5*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + aux},
                                {0.2*qd2 - 0.11*qd1*qd1*cos(q3) + 0.11*qd3*qd3*cos(q3) - 0.15*qd1*qd1*sin(2.0*q2) - 0.068*qd1*qd1*sin(2.0*q3) + 0.22*qd2*qd3*cos(q3) + 0.013*qd2*qd5*cos(q5) + 0.013*qd3*qd5*cos(q5) - 1.7e-3*qd4*qd5*sin(q4) + 0.22*qd1*qd1*cos(q2)*cos(q2)*cos(q3) + 8.4e-4*qd2*qd5*sin(2.0*q5) + 8.4e-4*qd3*qd5*sin(2.0*q5) - 6.5e-3*qd1*qd1*cos(q4)*cos(q5) + 6.5e-3*qd4*qd4*cos(q4)*cos(q5) + 6.5e-3*qd5*qd5*cos(q4)*cos(q5) - 4.6e-3*qd1*qd1*cos(q3)*sin(q5) + 4.6e-3*qd3*qd3*cos(q3)*sin(q5) + 4.6e-3*qd5*qd5*cos(q3)*sin(q5) + 1.5e-4*qd1*qd4*sin(q2)*sin(q3) - 0.013*qd4*qd5*sin(q4)*sin(q5) + 4.0e-6*qd5*qd6*sin(q4)*sin(q5) + 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 0.013*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.27*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2) + 0.27*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3) + 8.4e-4*qd1*qd1*cos(q2)*cos(q5)*cos(q5)*sin(q2) + 9.2e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q5) + 8.4e-4*qd1*qd1*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 1.7e-3*qd4*qd5*cos(q5)*cos(q5)*sin(q4) - 4.6e-3*qd1*qd1*cos(q4)*cos(q5)*sin(q3) - 8.4e-4*qd1*qd1*cos(q4)*cos(q5)*sin(q5) + 4.6e-3*qd3*qd3*cos(q4)*cos(q5)*sin(q3) + 4.6e-3*qd4*qd4*cos(q4)*cos(q5)*sin(q3) + 4.6e-3*qd5*qd5*cos(q4)*cos(q5)*sin(q3) + 8.4e-4*qd4*qd4*cos(q4)*cos(q5)*sin(q5) - 0.22*qd1*qd1*cos(q2)*sin(q2)*sin(q3) - 0.013*qd1*qd1*cos(q2)*sin(q2)*sin(q5) - 0.013*qd1*qd1*cos(q3)*sin(q3)*sin(q5) - 1.5e-4*qd1*qd4*cos(q2)*cos(q3) - 4.0e-6*qd4*qd6*cos(q4)*cos(q5) + 9.2e-3*qd2*qd3*cos(q3)*sin(q5) + 9.2e-3*qd2*qd5*cos(q5)*sin(q3) + 9.2e-3*qd3*qd5*cos(q5)*sin(q3) + 0.026*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) + 0.026*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) - 1.7e-3*qd2*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd3*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd2*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd3*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 0.026*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 8.4e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 8.4e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 9.2e-3*qd1*qd1*cos(q2)*sin(q2)*sin(q3)*sin(q5) + 9.2e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5) - 4.0e-6*qd1*qd6*cos(q2)*cos(q3)*sin(q5) + 9.2e-3*qd2*qd3*cos(q4)*cos(q5)*sin(q3) + 9.2e-3*qd2*qd4*cos(q3)*cos(q5)*sin(q4) + 9.2e-3*qd2*qd5*cos(q3)*cos(q4)*sin(q5) + 9.2e-3*qd3*qd4*cos(q3)*cos(q5)*sin(q4) + 9.2e-3*qd3*qd5*cos(q3)*cos(q4)*sin(q5) - 1.7e-3*qd1*qd5*cos(q2)*sin(q3)*sin(q4) - 1.7e-3*qd1*qd5*cos(q3)*sin(q2)*sin(q4) - 9.2e-3*qd1*qd5*cos(q2)*sin(q4)*sin(q5) + 4.0e-6*qd1*qd6*sin(q2)*sin(q3)*sin(q5) - 9.2e-3*qd4*qd5*sin(q3)*sin(q4)*sin(q5) + 9.2e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) + 1.7e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 1.7e-3*qd1*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd1*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 3.4e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 9.2e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.7e-3*qd1*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.013*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.013*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 4.0e-6*qd1*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 4.0e-6*qd1*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.013*qd1*qd5*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.013*qd1*qd5*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.026*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 3.4e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.7e-3*qd1*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)},
                                {0.2*qd3 - 0.11*qd1*qd1*cos(q3) - 0.11*qd2*qd2*cos(q3) - 0.068*qd1*qd1*sin(2.0*q2) - 0.068*qd1*qd1*sin(2.0*q3) + 0.013*qd2*qd5*cos(q5) + 0.013*qd3*qd5*cos(q5) - 1.7e-3*qd4*qd5*sin(q4) + 0.11*qd1*qd1*cos(q2)*cos(q2)*cos(q3) + 8.4e-4*qd2*qd5*sin(2.0*q5) + 8.4e-4*qd3*qd5*sin(2.0*q5) - 6.5e-3*qd1*qd1*cos(q4)*cos(q5) + 6.5e-3*qd4*qd4*cos(q4)*cos(q5) + 6.5e-3*qd5*qd5*cos(q4)*cos(q5) - 4.6e-3*qd1*qd1*cos(q3)*sin(q5) - 4.6e-3*qd2*qd2*cos(q3)*sin(q5) + 1.5e-4*qd1*qd4*sin(q2)*sin(q3) - 0.013*qd4*qd5*sin(q4)*sin(q5) + 4.0e-6*qd5*qd6*sin(q4)*sin(q5) + 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5) + 0.013*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5) + 0.27*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2) + 0.27*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3) + 8.4e-4*qd1*qd1*cos(q2)*cos(q5)*cos(q5)*sin(q2) + 4.6e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q5) + 8.4e-4*qd1*qd1*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 1.7e-3*qd4*qd5*cos(q5)*cos(q5)*sin(q4) - 4.6e-3*qd1*qd1*cos(q4)*cos(q5)*sin(q3) - 4.6e-3*qd2*qd2*cos(q4)*cos(q5)*sin(q3) - 8.4e-4*qd1*qd1*cos(q4)*cos(q5)*sin(q5) + 8.4e-4*qd4*qd4*cos(q4)*cos(q5)*sin(q5) - 0.11*qd1*qd1*cos(q2)*sin(q2)*sin(q3) - 0.013*qd1*qd1*cos(q2)*sin(q2)*sin(q5) - 0.013*qd1*qd1*cos(q3)*sin(q3)*sin(q5) - 1.5e-4*qd1*qd4*cos(q2)*cos(q3) - 4.0e-6*qd4*qd6*cos(q4)*cos(q5) + 0.026*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*sin(q2)*sin(q5) + 0.026*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*sin(q3)*sin(q5) - 1.7e-3*qd2*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd3*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd2*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd3*qd5*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 0.026*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5) - 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*cos(q5)*sin(q2) + 8.4e-4*qd1*qd1*cos(q2)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*cos(q5)*sin(q3) + 8.4e-4*qd1*qd1*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 4.6e-3*qd1*qd1*cos(q2)*sin(q2)*sin(q3)*sin(q5) - 4.0e-6*qd1*qd6*cos(q2)*cos(q3)*sin(q5) - 1.7e-3*qd1*qd5*cos(q2)*sin(q3)*sin(q4) - 1.7e-3*qd1*qd5*cos(q3)*sin(q2)*sin(q4) + 4.0e-6*qd1*qd6*sin(q2)*sin(q3)*sin(q5) + 4.6e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*sin(q5) + 1.7e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) + 1.7e-3*qd1*qd5*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd1*qd5*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 3.4e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 4.6e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 1.7e-3*qd1*qd4*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.013*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.013*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 4.0e-6*qd1*qd6*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 4.0e-6*qd1*qd6*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.013*qd1*qd5*cos(q2)*sin(q3)*sin(q4)*sin(q5) - 0.013*qd1*qd5*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.026*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd1*qd4*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd4*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 3.4e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.7e-3*qd1*qd5*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)},
                                {0.2*qd4 + 4.0e-6*qd5*qd6*cos(q5) + 5.2e-5*qd2*qd5*sin(q4) + 5.2e-5*qd3*qd5*sin(q4) - 8.4e-4*qd4*qd5*sin(2.0*q5) - 1.5e-4*qd1*qd2*sin(q2)*sin(q3) - 1.5e-4*qd1*qd3*sin(q2)*sin(q3) - 8.4e-4*qd1*qd1*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 8.4e-4*qd2*qd2*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 8.4e-4*qd3*qd3*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 1.7e-3*qd2*qd5*cos(q5)*cos(q5)*sin(q4) + 1.7e-3*qd3*qd5*cos(q5)*cos(q5)*sin(q4) - 4.6e-3*qd1*qd1*cos(q3)*cos(q5)*sin(q4) - 4.6e-3*qd2*qd2*cos(q3)*cos(q5)*sin(q4) + 1.5e-4*qd1*qd2*cos(q2)*cos(q3) + 1.5e-4*qd1*qd3*cos(q2)*cos(q3) + 4.0e-6*qd2*qd6*cos(q4)*cos(q5) + 4.0e-6*qd3*qd6*cos(q4)*cos(q5) + 1.7e-3*qd2*qd3*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 8.4e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q4) + 8.4e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 6.5e-3*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q4) - 6.5e-3*qd1*qd1*cos(q3)*cos(q5)*sin(q3)*sin(q4) - 9.2e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q5) + 5.2e-5*qd1*qd5*cos(q2)*cos(q3)*cos(q4) - 5.2e-5*qd1*qd5*cos(q4)*sin(q2)*sin(q3) + 4.6e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 1.7e-3*qd1*qd5*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) + 1.7e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) + 1.7e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*cos(q5) - 1.7e-3*qd1*qd2*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd1*qd3*cos(q4)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q4) - 4.6e-3*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 8.4e-4*qd1*qd1*cos(q2)*cos(q5)*sin(q2)*sin(q4)*sin(q5) - 8.4e-4*qd1*qd1*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) - 0.013*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 0.013*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 0.013*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 0.013*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2) - 4.0e-6*qd1*qd6*cos(q2)*cos(q3)*cos(q5)*sin(q4) - 1.7e-3*qd1*qd5*cos(q2)*cos(q5)*sin(q3)*sin(q5) - 1.7e-3*qd1*qd5*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 4.0e-6*qd1*qd6*cos(q5)*sin(q2)*sin(q3)*sin(q4) + 0.013*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4) + 1.7e-3*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) - 1.7e-3*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 1.7e-3*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) - 1.7e-3*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5) - 1.7e-3*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5) + 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q5)*sin(q3)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3)*sin(q4)},
                                {0.2*qd5 - 6.5e-3*qd1*qd1*cos(q5) - 6.5e-3*qd2*qd2*cos(q5) - 6.5e-3*qd3*qd3*cos(q5) - 4.2e-4*qd2*qd2*sin(2.0*q5) - 4.2e-4*qd3*qd3*sin(2.0*q5) + 4.2e-4*qd4*qd4*sin(2.0*q5) - 0.013*qd2*qd3*cos(q5) - 4.0e-6*qd4*qd6*cos(q5) - 5.2e-5*qd2*qd4*sin(q4) - 5.2e-5*qd3*qd4*sin(q4) + 6.5e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q5) + 6.5e-3*qd1*qd1*cos(q3)*cos(q3)*cos(q5) - 8.4e-4*qd2*qd3*sin(2.0*q5) - 4.6e-3*qd1*qd1*cos(q5)*sin(q3) - 4.6e-3*qd2*qd2*cos(q5)*sin(q3) - 4.0e-6*qd2*qd6*sin(q4)*sin(q5) - 4.0e-6*qd3*qd6*sin(q4)*sin(q5) + 4.6e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q5)*sin(q3) + 8.4e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q5)*sin(q5) + 8.4e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q5)*sin(q5) - 8.4e-4*qd1*qd1*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 8.4e-4*qd2*qd2*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 8.4e-4*qd3*qd3*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd2*qd4*cos(q5)*cos(q5)*sin(q4) - 1.7e-3*qd3*qd4*cos(q5)*cos(q5)*sin(q4) - 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5) - 8.4e-4*qd1*qd1*cos(q2)*cos(q4)*sin(q2) - 8.4e-4*qd1*qd1*cos(q3)*cos(q4)*sin(q3) - 4.6e-3*qd1*qd1*cos(q3)*cos(q4)*sin(q5) - 4.6e-3*qd2*qd2*cos(q3)*cos(q4)*sin(q5) + 1.7e-3*qd2*qd3*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q5)*sin(q5) + 8.4e-4*qd1*qd1*cos(q2)*cos(q2)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 8.4e-4*qd1*qd1*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) + 4.6e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2) - 6.5e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2)*sin(q5) - 6.5e-3*qd1*qd1*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 5.2e-5*qd1*qd4*cos(q2)*cos(q3)*cos(q4) - 4.0e-6*qd1*qd6*cos(q2)*cos(q5)*sin(q3) - 4.0e-6*qd1*qd6*cos(q3)*cos(q5)*sin(q2) + 1.7e-3*qd1*qd2*cos(q2)*sin(q3)*sin(q4) + 1.7e-3*qd1*qd2*cos(q3)*sin(q2)*sin(q4) + 1.7e-3*qd1*qd3*cos(q2)*sin(q3)*sin(q4) + 1.7e-3*qd1*qd3*cos(q3)*sin(q2)*sin(q4) + 9.2e-3*qd1*qd2*cos(q2)*sin(q4)*sin(q5) + 5.2e-5*qd1*qd4*cos(q4)*sin(q2)*sin(q3) + 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2) + 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3) + 1.7e-3*qd1*qd1*cos(q2)*cos(q4)*cos(q5)*cos(q5)*sin(q2) + 4.6e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 1.7e-3*qd1*qd1*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) - 1.7e-3*qd1*qd2*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) - 1.7e-3*qd1*qd2*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) - 1.7e-3*qd1*qd3*cos(q2)*cos(q5)*cos(q5)*sin(q3)*sin(q4) - 1.7e-3*qd1*qd3*cos(q3)*cos(q5)*cos(q5)*sin(q2)*sin(q4) + 1.7e-3*qd1*qd4*cos(q4)*cos(q5)*cos(q5)*sin(q2)*sin(q3) - 3.4e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q2) - 3.4e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5)*sin(q3) + 0.013*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3) - 1.7e-3*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q5) - 4.6e-3*qd1*qd1*cos(q2)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 4.0e-6*qd1*qd6*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 1.7e-3*qd1*qd4*cos(q2)*cos(q5)*sin(q3)*sin(q5) + 1.7e-3*qd1*qd4*cos(q3)*cos(q5)*sin(q2)*sin(q5) + 0.013*qd1*qd2*cos(q2)*sin(q3)*sin(q4)*sin(q5) + 0.013*qd1*qd2*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 0.013*qd1*qd3*cos(q2)*sin(q3)*sin(q4)*sin(q5) + 0.013*qd1*qd3*cos(q3)*sin(q2)*sin(q4)*sin(q5) + 4.0e-6*qd1*qd6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 0.013*qd1*qd1*cos(q2)*cos(q3)*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 0.013*qd1*qd1*cos(q2)*cos(q2)*cos(q3)*cos(q4)*sin(q3)*sin(q5) - 1.7e-3*qd1*qd4*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q5) + 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*sin(q5) - 1.7e-3*qd1*qd2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) - 1.7e-3*qd1*qd3*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd3*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 1.7e-3*qd1*qd1*cos(q2)*cos(q3)*cos(q4)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q5)},
                                {0.2*qd6 + 4.0e-6*qd4*qd5*cos(q5) + 4.0e-6*qd2*qd5*sin(q4)*sin(q5) + 4.0e-6*qd3*qd5*sin(q4)*sin(q5) - 4.0e-6*qd2*qd4*cos(q4)*cos(q5) - 4.0e-6*qd3*qd4*cos(q4)*cos(q5) + 4.0e-6*qd1*qd2*cos(q2)*cos(q3)*sin(q5) + 4.0e-6*qd1*qd3*cos(q2)*cos(q3)*sin(q5) + 4.0e-6*qd1*qd5*cos(q2)*cos(q5)*sin(q3) + 4.0e-6*qd1*qd5*cos(q3)*cos(q5)*sin(q2) - 4.0e-6*qd1*qd2*sin(q2)*sin(q3)*sin(q5) - 4.0e-6*qd1*qd3*sin(q2)*sin(q3)*sin(q5) + 4.0e-6*qd1*qd2*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 4.0e-6*qd1*qd2*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 4.0e-6*qd1*qd3*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 4.0e-6*qd1*qd3*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 4.0e-6*qd1*qd4*cos(q2)*cos(q3)*cos(q5)*sin(q4) + 4.0e-6*qd1*qd5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 4.0e-6*qd1*qd4*cos(q5)*sin(q2)*sin(q3)*sin(q4) - 4.0e-6*qd1*qd5*cos(q4)*sin(q2)*sin(q3)*sin(q5)}
                                };
            double Ga[6][6] = {
                                {                                                                                                                                                                                                        0},
                                {g*(0.97*cos(q2)*cos(q3) - 1.7*sin(q2) - 0.97*sin(q2)*sin(q3) - 0.04*sin(q2)*sin(q3)*sin(q5) + 0.04*cos(q2)*cos(q3)*sin(q5) + 0.04*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.04*cos(q3)*cos(q4)*cos(q5)*sin(q2))},
                                {              g*(0.97*cos(q2)*cos(q3) - 0.97*sin(q2)*sin(q3) - 0.04*sin(q2)*sin(q3)*sin(q5) + 0.04*cos(q2)*cos(q3)*sin(q5) + 0.04*cos(q2)*cos(q4)*cos(q5)*sin(q3) + 0.04*cos(q3)*cos(q4)*cos(q5)*sin(q2))},
                                {                                                                                                                                                                      0.04*g*cos(q2 + q3)*cos(q5)*sin(q4)},
                                {                                                            g*(0.04*cos(q2)*cos(q5)*sin(q3) + 0.04*cos(q3)*cos(q5)*sin(q2) + 0.04*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 0.04*cos(q4)*sin(q2)*sin(q3)*sin(q5))},
                                {                                                                                                                                                                                                        0}
                                };

            double aux3 = 0;
            for (unsigned int i = 0; i < 6; i++){
                for (int k = 0; k < 6; k++){
                    aux3 = aux3 + Ma[i][k]*v[k][0];
                }
                torque_[i][0] = Va[i][0] + Ga[i][0] + aux3;
                aux3 = 0;
            }

            torque[0][0] = torque_[2][0];   // elbow
            torque[1][0] = torque_[1][0];   // shoulder lift
            torque[2][0] = torque_[0][0];   // shoulder pan

            for (unsigned int i = 3; i < 6; i++){                
                torque[i][0] = torque_[i][0];
            }

            joints_[0].setCommand(copysign(std::min(abs(torque[0][0]), 50.0), torque[0][0]));
            joints_[1].setCommand(copysign(std::min(abs(torque[1][0]), 60.0), torque[1][0]));
            joints_[2].setCommand(copysign(std::min(abs(torque[2][0]), 20.0), torque[2][0]));
            joints_[3].setCommand(copysign(std::min(abs(torque[3][0]), 10.0), torque[3][0]));
            joints_[4].setCommand(copysign(std::min(abs(torque[4][0]), 10.0), torque[4][0]));

            error = qr[5][0] - joints_[5].getPosition();
            U = error*10;   
            joints_[5].setCommand(copysign(std::min(abs(U), 10.0), U));


            current_time = ros::Time::now();

            msg1.header.stamp = current_time;
            msg1.pose.position.x = q_[0][0];
            msg1.pose.position.y = qr_[0][0];
            msg1.pose.position.z = dq_[0][0];
            msg1.pose.orientation.x = dqr_[0][0];
            msg1.pose.orientation.y = torque_[0][0];
            pub_graficas1.publish(msg1);

            msg2.header.stamp = current_time;
            msg2.pose.position.x = q_[1][0];
            msg2.pose.position.y = qr_[1][0];
            msg2.pose.position.z = dq_[1][0];
            msg2.pose.orientation.x = dqr_[1][0];
            msg2.pose.orientation.y = torque_[1][0];
            pub_graficas2.publish(msg2);

            msg3.header.stamp = current_time;
            msg3.pose.position.x = q_[2][0];
            msg3.pose.position.y = qr_[2][0];
            msg3.pose.position.z = dq_[2][0];
            msg3.pose.orientation.x = dqr_[2][0];
            msg3.pose.orientation.y = torque_[2][0];
            pub_graficas3.publish(msg3);

            msg4.header.stamp = current_time;
            msg4.pose.position.x = q_[3][0];
            msg4.pose.position.y = qr_[3][0];
            msg4.pose.position.z = dq_[3][0];
            msg4.pose.orientation.x = dqr_[3][0];
            msg4.pose.orientation.y = torque_[3][0];
            pub_graficas4.publish(msg4);

            msg5.header.stamp = current_time;
            msg5.pose.position.x = q_[4][0];
            msg5.pose.position.y = qr_[4][0];
            msg5.pose.position.z = dq_[4][0];
            msg5.pose.orientation.x = dqr_[4][0];
            msg5.pose.orientation.y = torque_[4][0];
            pub_graficas5.publish(msg5);

            msg6.header.stamp = current_time;
            msg6.pose.position.x = q_[5][0];
            msg6.pose.position.y = qr_[5][0];
            msg6.pose.position.z = dq_[5][0];
            msg6.pose.orientation.x = dqr_[5][0];
            msg6.pose.orientation.y = torque_[5][0];
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

            qr_[0][0] = 0;
            qr_[1][0] = 0;
            qr_[2][0] = 0;
            qr_[3][0] = 0;
            qr_[4][0] = 0;
            qr_[5][0] = 0;
                       
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
            std::string name_;               // Controller name.

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