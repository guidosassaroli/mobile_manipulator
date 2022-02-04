cmake_minimum_required(VERSION 3.1.3)
project(ommp_lisa_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  joint_state_publisher
  robot_state_publisher
  rospy
  rviz
  controller_interface
  hardware_interface
  pluginlib
  roscpp
  actionlib
)

catkin_package(
  DEPENDS roscpp gazebo_ros actionlib
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
# include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )