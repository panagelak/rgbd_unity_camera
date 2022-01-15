# rgbd Unity Camera to ROS 

## Overview

This repository holds the Unity Project that simulates an rgbd camera meaning that it publishes into Ros 4 topics

```bash
# rgb compress image
/unity_camera/rgb/image_raw/compressed 
# rgb camera info 
/unity_camera/rgb/camera_info
# depth compressed image (32FC1)
/unity_camera/depth/image_raw/compressed
# depth camera info
/unity_camera/depth/camera_info
```


Then ROS subscribes to this topics and can decompress the rgb and depth images **(using different decompression libraries)** into the following topics

```bash
# rgb image
/unity_camera/rgb/image_raw 
# depth image
/unity_camera/depth/image_raw
```

By combining the rgb + depth images and the camera info it is possible to extract a colored pointcloud into ros. The current depth_image_to_pc2 library can only combine depth + cam info to publish a non colored pointcloud.


On the **image_transfer** folder i have the libraries that can compress/decompress rgb and depth images and construct a non colored pointcloud by combining depth + cam info.

**This libraries have been taken and modified from the image_transport ros package (i made them into libraries for a simple implementation)**

**See the unity_compress_subscriber**

### Help NEEDED

I would like a pull request where someone fixes the Unity scripts in order to publish correctly the compress depth image with format **32FC1**

**I have also included some scripts from this repo** 
- [Unity Depth Camera Simulation](https://www.immersivelimit.com/tutorials/unity-depth-camera-simulation)

However the format of the depth image it construct through the shader is rgb format instead of the 32FC1 which i was able to compress like a normal **rgb** image and send it to Ros. However this is not a proper ros depth image.

Missing implementation

1. **Create Unity Shader that gives 32FC1 format**
2. **Create Encode function that compresses this depth image (in the same way as ros)**

### Requirements

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

- Unity 2020.3.25

    It is recommended to use **Ubuntu 20.04 with ROS noetic**


#### Building procedure for ROS

```bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# Clone the latest version of this repository 
git clone https://github.com/panagelak/rgbd_unity_camera

# Install dependencies of all packages.
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build the workspace
catkin_make

# activate the workspace
source devel/setup.bash
```

### Launch

```bash
# configure tcp ip for unity robotics hub
# at unity_bringup/config/config.yaml

# MAIN Launch file (change the ip for the ros-tcp-endpoint) 
# and in Unity from the Robotics panel
roslaunch unity_bringup unity_bringup.launch ROS_IP:=192.168.2.7
```