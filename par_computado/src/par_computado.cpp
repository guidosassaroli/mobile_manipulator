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

//include amjack0 
#include "actionlib/server/simple_action_server.h"


namespace par_computado_ns
{

    class ParComputado : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        //this is going to be loaded by the controller manager

        KDL::ChainIdSolver_RNE *idsolver;
        KDL::JntArray q;
        KDL::JntArray dq;
        KDL::JntArray v;
        KDL::JntArray qr;
        KDL::JntArray dqr;
        KDL::JntArray ddqr;
        KDL::JntArray torque;
        KDL::Wrenches fext;
        Eigen::MatrixXd Kp;
        Eigen::MatrixXd Kd;

        void commandCB(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint);

        public:
        ParComputado(void);
        ~ParComputado(void);

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            //get the joint that we want to control

            XmlRpc::XmlRpcValue joint_names;
            if(!n.getParam("joints",joint_names))
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

/*             command_ = joint_.getPosition();  // set the current joint goal to the current INITIAL joint position
 */
            //every time a new command arrives it runs the callback
/*             sub_command_ = n.subscribe<std_msgs::Float64>("command", 1000, &ParComputado::setCommandCB, this);*/
            
            sub_command_=n.subscribe("command",1000, &ParComputado::commandCB, this);

            std::string robot_desc_string;
            if(!n.getParam("/robot_description",robot_desc_string))
            {
            ROS_ERROR("Could not find ’/robot_description’.");
            return false;
            }

            KDL::Tree tree;
            if (!kdl_parser::treeFromString(robot_desc_string,tree))
            {
            ROS_ERROR("Failed to construct KDL tree.");
            return false;
            }
            KDL::Chain chain;
            if (!tree.getChain("arm_base","wrist_3_link",chain))
            {
            ROS_ERROR("Failed to get chain from KDL tree.");
            return false;
            }
            KDL::Vector g;
            n.param("/gazebo/gravity_x",g[0],0.0);
            n.param("/gazebo/gravity_y",g[1],0.0);
            n.param("/gazebo/gravity_z",g[2],-9.8);
            if((idsolver=new KDL::ChainIdSolver_RNE(chain,g)) == NULL)
            {
            ROS_ERROR("Failed to create ChainIDSolver_RNE.");
            return false;
            }

            q.resize(chain.getNrOfJoints());
            dq.resize(chain.getNrOfJoints());
            v.resize(chain.getNrOfJoints());
            qr.resize(chain.getNrOfJoints());
            dqr.resize(chain.getNrOfJoints());
            ddqr.resize(chain.getNrOfJoints());
            torque.resize(chain.getNrOfJoints());
            fext.resize(chain.getNrOfSegments());
            Kp.resize(chain.getNrOfJoints(),chain.getNrOfJoints());
            Kd.resize(chain.getNrOfJoints(),chain.getNrOfJoints());

            return true;
        }

        // this loop is going to do the control
        void update(const ros::Time &time, const ros::Duration &duration)
        {
            for(unsigned int i=0;i < joints_.size();i++)
            {
            q(i)=joints_[i].getPosition();
            dq(i)=joints_[i].getVelocity();
            }
            for(unsigned int i=0;i < fext.size();i++) fext[i].Zero();
            v.data=ddqr.data+Kp*(qr.data-q.data)+Kd*(dqr.data-dq.data);
            if(idsolver->CartToJnt(q,dq,v,fext,torque) < 0)
            ROS_ERROR("KDL inverse dynamics solver failed.");
            for(unsigned int i=0;i < joints_.size();i++)
            joints_[i].setCommand(torque(i));
        }

        // update the command with the trajectory
/*         void setCommandCB(const std_msgs::Float64ConstPtr &msg)
        {
            command_ = msg->data;
        }

         void traj_sub_callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
        {
            traj = msg->trajectory[0].joint_trajectory;
        } */

        void starting(const ros::Time &time) {

            Kp.setZero();
            Kd.setZero();
            for(unsigned int i=0;i < joints_.size();i++)
            {
            Kp(i,i)=Wn*Wn;
            Kd(i,i)=2.0*Xi*Wn;
            q(i)=joints_[i].getPosition();
            dq(i)=joints_[i].getVelocity();
            }
            qr=q;
            dqr=dq;
            SetToZero(ddqr);
            struct sched_param param;
            param.sched_priority=sched_get_priority_max(SCHED_FIFO);
            if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
            {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
            }
/*             if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
            ROS_WARN("Failed to lock memory."); */
        }
        void stopping(const ros::Time &time) {}

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            double Wn = 1;
            double Xi = 1;
            trajectory_msgs::JointTrajectory traj;
            ros::Subscriber sub_command_;
            ros::Subscriber traj_sub;
            std::vector<hardware_interface::JointHandle> joints_;

            


    };
    PLUGINLIB_EXPORT_CLASS(par_computado_ns::ParComputado, controller_interface::ControllerBase);
}