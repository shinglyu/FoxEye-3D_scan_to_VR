#! /bin/sh
source /opt/ros/indigo/setup.sh        
source ~/catkin_ws/devel/setup.zsh # for realsense_camera package

# You need to run the following two commands in separate consoles
# roscore # start ros master
# roslaunch realsense_camera realsense_rviz.launch # start the camera and publish the data to topics

rosbag record --output-name=beanbag.bag --duration=3 /camera/depth/camera_info /camera/depth/image_raw /camera/depth/points /camera/ir/image_raw /camera/ir/camera_info /tf /tf_static

rosrun pcl_ros bag_to_pcd beanbag.bag /camera/depth/points ./output
# Use the following command to check
# pcl_viewer output/1456306720.889269257.pcd
