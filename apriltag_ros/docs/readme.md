# Project Overview  

This project is an early-stage system aiming to integrate ROS with the GulliView system.  
The goal is to create a more structured and efficient solution that can potentially be used in **Chalmers Lab 5355**.  

Currently, an original GulliView system is running, and this project seeks to improve upon it.  
Note: This system is still under development and has not been extensively tested.  

This project is based on [apritag_ros](https://github.com/AprilRobotics/apriltag_ros), which provide powerful API to make apritag detection.

![System Diagram](https://github.com/chuanchuan-Dong/gulliview_noetic/blob/main/apriltag_ros/docs/system_overview.png)  
## Gulliview_Noetic
- [x] Global Coordination System(cemtimeter error)
- [x] Vehicle Orientation (3-axis)
- [x] Self-adjusted Vehicle Height Influence  
- [x] Prepared for Navigation(Nav_msgs::odometry msg type avaibale at guard node)
- [ ] Nodelet
- [ ] Bundle tags
- [ ] Nvidia support

# DEMO
### Accrossing 4 zones
![Project Demo](https://github.com/chuanchuan-Dong/gulliview_noetic/blob/main/apriltag_ros/docs/demo_gullview-noetic.gif)  

### Navigation(collaborate with vehicle)
![Project Demo](https://github.com/chuanchuan-Dong/gulliview_noetic/blob/main/apriltag_ros/docs/demo2_nav.gif)  

### Setup

Manually store testbed image attached with floortag when make a change. e.g. /gulliview_noetic/apritag_ros/config/data/camx.jpg

Also change the path on global_client launch file.


### RUN!!!!

1. roslaunch apriltag_ros global_server.launch
2. roslaunch apriltag_ros global_client.launch
3. roslaunch apriltag_ros 4camera_stream_detection.launch
4. roslaunch apriltag_ros 4object_global.launch
5. rosrun apriltag_ros gulliview_noetic_object_pose_guard
