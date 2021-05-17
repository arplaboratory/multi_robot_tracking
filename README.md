# PHD_Filter
Multi Agent Tracking for Data Association

## Overview
This is ROS package developed for tracking multiple agents using a camera. This software is designed for associating 2D bounding box measurements (of drones) to unique IDs and 2D position in image space. The required inputs for this software are 2d bounding box measurements and RGB image. The provided outputs are 2D positions with target associations. The motion model is based on Gaussian Linear kalman filter with probabilistic hypothesis density (PHD) filter to solve association. Can also run Joint Probability Association filter (JPDAF) by switching rosparam upon launch. This repository includes matlab implementation and evaluation as well.    

'Input rostopic: /darknet_ros/bounding_boxes (Type: darknet_ros_msgs/BoundingBoxes)<br />
'Output rostopic: /phd_tracker/tracked_image (Type: sensor_msgs/Image)<br />

![Screenshot](doc/tracking.png)


**Developer: Mark Lee<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: Mark Lee, ml7617@nyu.edu<br />**


## Install
The tracking filter package is dependent on Eigen and Boost, and ROS. The additional repo can be installed below:

install darknet ros for 2d bounding box test (older version to be compatiable with rosbag data)
```
$ git clone https://github.com/ShiSanChuan/darknet_ros.git
$ cd ..
$ catkin_make darknet_ros -DCMAKE_BUILD_TYPE=Release
$ source devel/setup.bash
```

install filter repository into catkin directory
```
$ git clone https://github.com/arplaboratory/multi_robot_tracking.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

retrieve rosbag data from [ARPL data folder](https://drive.google.com/drive/folders/1xc6DbgBbhABoLlvGTSrrJ1zFWL4S-ZTt?usp=sharing) after gaining access

## Running from Simulation Data (Matlab)
Move the corresponding rosbag data to the Matlab directory. Make sure the GM_PHD_Initialisation_drones.m file points to the correct directory for rosbag data. 

## Running from Simulation Data (ROS, C++)
Move rosbag data into corresponding directory. Modify the rostopic subscriber in phd_tracker.cpp if wanting to modify using a different measured input.
```
$ roscore
$ rosbag play darknet_detection_3drones_VICON_TobiiGlasses.bag 
$ rosrun phd_tracker phd_tracker

```

## Running from Simulation Data (ROS, Flightmare)
We can also utilize the Flightare simulator for photorealistic rendering to create new data. Refer to [Flightmare github page](https://github.com/uzh-rpg/flightmare) for installation and setting up dependencies. <br />
Running the below code launches Flightmare simulation with Gazebo for multiple drones, multimotion creates trajectories for all drones, and phd_tracker associates each target in space.
```
$ source ~/flightMare_sim_ws/devel/setup.bash
$ roscd flightros/launch/multi_tracking/
$ roslaunch multi_robot.launch 
$ rosrun flightros multimotion
$ rosrun phd_tracker phd_tracker
```
