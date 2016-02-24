#! /bin/sh
source /opt/ros/indigo/setup.sh        
# roscore
source ~/catkin_ws/devel/setup.zsh
roslaunch realsense_camera realsense_rviz.launch debug_depth_unit:=true
