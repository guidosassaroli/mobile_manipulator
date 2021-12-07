# SCHEME OF THE REPOSITORY 

## launch tree 

1- basic.launch 

2- ommp_bringup/launch/sim_bringup.launch
2- gazebo_ros/launch/empty_world.launch 
2- robot_state_publisher, controller_spawner, controller_manager_group, robot_loalization 

3- ommp_moveit_interface/launch/moveit.launch
3- ommp_sim_moveit_config/launch/move_group.launch
3- ommp_real_moveit_config/launch/move_group.launch

4- moveit node 

5- ommp_navigation/launch/navigation_main.launch
Run the map server and load the map file 
5- ommp_navigation/launch/include/amcl.launch
5- ommp_navigation/launch/include/amcl.launch AMCL params 

6- ommp_navigation/launch/include/move_base.launch 

7- rviz node 




## to_arduino.ino 

### ROS parameters 

#### ROS Initilization
The arduino file publishes the *encoder_ticks* as a Int64MultiArray and the *velocity_wheeles* as a Float64MultiArray.

#### We set the PID callback
We have a PID controller for heach wheel. 

#### We set the cmd_vel callback

It subscribes to the *set_vel* and to te *pid_set*




