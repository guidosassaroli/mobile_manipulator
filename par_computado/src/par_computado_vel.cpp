// //CODE: https://github.com/amjack0/custom_ros_action_server

// //NON FUNZIONA

// #include "ros/ros.h"
// #include <iostream>
// #include "actionlib/server/simple_action_server.h"
// // #include "ros_action_server/MyMsgAction.h"

// #include "std_msgs/Empty.h"
// #include "std_msgs/Bool.h"
// #include "std_msgs/String.h"
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
// #include "sensor_msgs/JointState.h"
// #include "math.h"
// #include <array>
// #include <kdl/jntarray.hpp>

// using namespace par_computado_ns;
// #define N_JOINT 6


// class MoveRobotAction{

// protected:
//   ros::NodeHandle nh;
//   actionlib::SimpleActionServer<ros_action_server::MyMsgAction> action_server;
//   std::string action_name;                    // name of my action server
//   ros_action_server::MyMsgFeedback feedback; // variables stores the feedback
//   ros_action_server::MyMsgResult result;    // variables stores the result

//   //defining my subscribers

//   ros::Subscriber pos_sub;
//   sensor_msgs::JointState pos_info;

//   //defining my publishers

//   ros::Publisher tau_pub;
//   std::vector<ros::Publisher> tau_multiple_pub;


// public:
//   //constructor
//   MoveRobotAction(std::string name) :
//     action_server(nh, name, boost::bind(&MoveRobotAction::actionCb, this, _1),false),
//     action_name(name)
//   {
//    initializeVariables();
//    initializeSubscribers();
//    initializePublishers();
//    action_server.start();
//   }

// //   ~MoveRobotAction(void){} // TODO: delete

// private:

//   // provide your controller topic names
//   std::string tauTopicNames[N_JOINT] = {
//     "/p_c/endeffector_frc_trq_controller_1/command",
//     "/p_c/endeffector_frc_trq_controller_2/command",
//     "/p_c/endeffector_frc_trq_controller_3/command",
//     "/p_c/endeffector_frc_trq_controller_4/command",
//     "/p_c/endeffector_frc_trq_controller_5/command",
//     "/p_c/endeffector_frc_trq_controller_6/command",
//   };

//   std::array<int, N_JOINT> map_joint_states;

//   void initializeVariables(void)
//   {
//     map_joint_states={2, 1, 0, 3, 4, 5};
//   }

//   // initialize subscriber
//   void initializeSubscribers(void)
//   {
//     pos_sub = nh.subscribe("/joint_states", 1, &MoveRobotAction::subscriberCb, this);  //TODO:  1
//     ROS_INFO("[AS] Subscriber Initialized");
//   }

//   // initialize publisher

//   void initializePublishers(void)
//   {
//     //tau_pub = nh.advertise<std_msgs::Float64>("/urbot/endeffector_frc_trq_controller_2/command", 1);
//     for (short int j = 0; j < N_JOINT; j++)
//     {
//       tau_pub = nh.advertise<std_msgs::Float64>(tauTopicNames[j], 1); //TODO: 1
//       tau_multiple_pub.push_back(tau_pub);
//     }
//     ROS_INFO("[AS] Publisher Initialized");
//   }

//   void subscriberCb(const sensor_msgs::JointStateConstPtr &msg)
//   {
//     //clear pose info
//     pos_info.position.clear(); pos_info.effort.clear();
//     pos_info.position.resize(N_JOINT); pos_info.effort.resize(N_JOINT);

//     for(short int l=0; l< N_JOINT; l++){

//       pos_info.position.push_back( msg->position[map_joint_states[l]] );
//       pos_info.effort.push_back( msg->effort[map_joint_states[l]] );
//       //cout << "[AS] position: " <<  pos_info.position[l] << endl;
//     }
//     ROS_INFO("[AS] Reading joint_states");
//   }


//   // function to calculate the tau error and decides the feedback

//   std_msgs::Float64MultiArray calError(sensor_msgs::JointState current, const ros_action_server::MyMsgGoalConstPtr &goal)
//   {
//     std_msgs::Float64MultiArray err;    //TODO : reset the error
//     err.data.resize(N_JOINT);

//     for(short int l=0; l< N_JOINT; l++){
//       err.data[l] = abs(goal->tau.data[l])  - abs(current.effort[l]) ; // error = desired - current
//     }
//     return err;
//   }


//   //  main action server callback

//   void actionCb(const ros_action_server::MyMsgGoalConstPtr &goal)
//   {
//     ros::Rate rate(50);
//     bool success = true;
//     ROS_INFO("[AS] executing the action call_back");

//     std::vector<std_msgs::Float64> desired_tau(N_JOINT);
//     std_msgs::Float64 a;

//     for (short int j = 0; j < N_JOINT; j++){
//       a.data=static_cast<float>(goal->tau.data[j]);
//       desired_tau[j].data = a.data;
//       //cout << "desired_tau: " << desired_tau[j].data << endl;
//     }

//     do
//     {
//       for (short int j = 0; j < N_JOINT; j++)
//       {
//         tau_multiple_pub[j].publish(desired_tau[j]);
//       }
//       feedback.error = calError(pos_info, goal);
//       action_server.publishFeedback(feedback);

//       // take care of preemption
//       if(action_server.isPreemptRequested() || !ros::ok() )
//       {

//         for (short int j = 0; j < N_JOINT; j++){
//           a.data=static_cast<float>(0.0);     // robot will fall down !
//           desired_tau[j].data = a.data;
//         }

//         for (short int j = 0; j < N_JOINT; j++)
//         {
//           tau_multiple_pub[j].publish(desired_tau[j]);
//         }

//         ROS_INFO("[AS] %s: Preempted", action_name.c_str());
//         action_server.setPreempted();
//         success = false;
//         break;
//       }
//       rate.sleep();
//     }

//     // TODO: set the goal tolerance
//     while ( abs(feedback.error.data[0]) > 2 && abs(feedback.error.data[1]) > 2 && abs(feedback.error.data[2]) > 2 && abs(feedback.error.data[3]) > 2 && abs(feedback.error.data[4]) > 2 && abs(feedback.error.data[5]) > 2 );

//     // check if succeeded--yes-->return result
//     if(success)
//     {
//       result.status = "FINISHED";
//       ROS_INFO("[AS] %s: Succeeded", action_name.c_str());
//       action_server.setSucceeded(result); //! set the action state to succeeded
//     }
//   }
// };


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "action_server");
//   MoveRobotAction robot("trajectory_action");
//   ros::Rate rate(10);     // 10
//   unsigned int n=0;
//   while (ros::ok())
//   {
//     ros::spinOnce();
//     //ROS_INFO("%d" ,n); n++;
//     rate.sleep();
//   }
//   return 0;
// } 