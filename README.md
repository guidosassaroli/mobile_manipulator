# SCHEME OF THE REPOSITORY 

Credits for the model to https://github.com/panagelak. 
His project: https://github.com/panagelak/Open_Mobile_Manipulator.


# Compiling

mkdir -p ~/catkin_ws/src && cd catkin_ws/src

git clone https://github.com/guidosassaroli/mobile_manipulator 

cd ~/catkin_ws

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



# How to get data from the simulation

rostopic echo /odometry/filtered -p > odom.csv

rostopic echo /_base/TrajectoryPlannerROS/global_plan -p > trayect.csv

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped 'der: {stamp: now, frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'





