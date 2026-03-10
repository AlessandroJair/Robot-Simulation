cmake_minimum_required(VERSION 3.0.2)
project(rocker_bogie)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  gazebo_ros_control
  xacro
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros gazebo_ros_control xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
