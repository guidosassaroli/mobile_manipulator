# SCHEME OF THE REPOSITORY 

Credits for the model to https://github.com/panagelak. 
His project: https://github.com/panagelak/Open_Mobile_Manipulator.

# Our job 
We have designed and implemented a Computer Torque Control node for the manipulator. 
We have designed and implemented a Pure Pursuit Control for the mobile platform. 


# Compiling

mkdir -p ~/catkin_ws/src && cd catkin_ws/src

git clone https://github.com/guidosassaroli/mobile_manipulator 

cd ~/catkin_ws

change the name of the cloned folder from "mobile_manipulator" to "Open_Mobile_Manipulator"

rosdep install --from-paths src --ignore-src -r -y (install depedencies)

catkin_make sometimes doesn't work so build with catkin build **install it from install catkin tools

catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1

Now theoritically it should be easy to compile the project in ROS Melodic

# Launch sequence for the mobile robot controlled by the Pure Pursuit and manipulator controlled by the Computed Torque Control

roslaunch ommp_bringup complete.launch world:=jackal_race robot:=ommp_sim

roslaunch ommp_moveit_interface moveit.launch

roslaunch ommp_navigation navigation_main.launch map:=jackal_race

roslaunch ommp_viz rviz.launch config:=todo

rosrun pure_pursuit pure_pursuit


## launch sequence for the maniupulator controlled by the Computed Torque Control 

roslaunch ommp_lisa2_moveit_config gazebo.launch

roslaunch ommp_moveit_interface moveit.launch

roslaunch ommp_lisa2_moveit_config moveit_rviz.launch



# Launch sequence for the mobile robot controlled by the Pure Pursuit 

roslaunch ommp_bringup sim_bringup.launch world:=jackal_race robot:=ommp_sim

roslaunch ommp_navigation navigation_main.launch map:=jackal_race

roslaunch ommp_viz rviz.launch config:=navigation map:=jackal_race

rosrun pure_pursuit pure_pursuit



# How to get data from the simulation Pure Pursuit

First load the Pure Pursuit

rostopic echo /odometry/filtered -p > odom.csv

rostopic echo /_base/TrajectoryPlannerROS/global_plan -p > trayect.csv

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped 'der: {stamp: now, frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'


# How to get data from the simulation Pure Pursuit

First load the Par Computado

rostopic echo /arm_controller/q1_dq1 -p > q1_dq1.csv
rostopic echo /arm_controller/q2_dq2 -p > q2_dq2.csv
rostopic echo /arm_controller/q3_dq3 -p > q3_dq3.csv
rostopic echo /arm_controller/q4_dq4 -p > q4_dq4.csv
rostopic echo /arm_controller/q5_dq5 -p > q5_dq5.csv
rostopic echo /arm_controller/q6_dq6 -p > q6_dq6.csv





