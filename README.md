# SCHEME OF THE REPOSITORY 

## launch tree 

1) basic.launch 

2) ommp_bringup/launch/sim_bringup.launch
3) gazebo_ros)/launch/empty_world.launch 
4) robot_state_publisher, controller_spawner, controller_manager_group, robot_loalization 

5) ommp_moveit_interface/launch/moveit.launch
6) ommp_sim_moveit_config/launch/move_group.launch
7) ommp_real_moveit_config/launch/move_group.launch

8) moveit node 

9) ommp_navigation/launch/navigation_main.launch
Run the map server and load the map file 
10) ommp_navigation/launch/include/amcl.launch
11) ommp_navigation/launch/include/amcl.launch AMCL params 

12) ommp_navigation/launch/include/move_base.launch 

13) rviz node 




## to_arduino.ino 

### ROS parameters 

#### ROS Initilization
The arduino file publishes the *encoder_ticks* as a Int64MultiArray and the *velocity_wheeles* as a Float64MultiArray.

#### We set the PID callback
We have a PID controller for heach wheel. 

#### We set the cmd_vel callback

It subscribes to the *set_vel* and to te *pid_set*




