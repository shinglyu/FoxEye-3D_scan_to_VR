This package if for recording pcd files from Intel RealSense camera on Linux

!Installation

```
sudo apt-get install libudev-dev libv4l-dev
```

Install ROS (see BKM)

!Run
Run the scripts in the following order
1. `init_evn.sh # need to run in every terminal`
2. `roscore`
3. `start_camera.sh`
4. `record_pcd_ros.sh`

The output will be in `output/`
