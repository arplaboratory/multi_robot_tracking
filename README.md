# Multi Agent Tracking 
Multi Agent Tracking for Data Association with various filter comparison

## License
Please be aware that this code was originally implemented for research purposes and may be subject to changes and any fitness for a particular purpose is disclaimed. To inquire about commercial licenses, please contact Prof. Giuseppe Loianno (loiannog@nyu.edu).
```
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
```
## Citation
If you publish a paper with this work, please cite our paper: 
```
@article{VisualTraIROS2022,
  url = {https://arxiv.org/abs/2207.08301},
  Year = {2022},
  Booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}
  author = {Ge, Rundong and Lee, Moonyoung and Radhakrishnan, Vivek and Zhou, Yang and Li, Guanrui and Loianno, Giuseppe},
  title = {Vision-based Relative Detection and Tracking for Teams of Micro Aerial Vehicles}}
 ```

## Overview
This is ROS package developed for tracking multiple agents using a camera. This software is designed for associating 2D bounding box measurements (of drones) to unique IDs and 2D position in image space. The required inputs for this software are 2d bounding box measurements and RGB image. The provided outputs are 2D positions with target associations. The motion model is based on Gaussian Linear kalman filter with probabilistic hypothesis density (PHD) filter to solve association. Can also run Joint Probability Association filter (JPDAF) by switching rosparam upon launch. This repository includes matlab implementation and evaluation as well.    

![Screenshot](doc/tracking.png)


**Developer: Mark Lee, Vivek Radhakrishnan<br />
Affiliation: [NYU ARPL](https://wp.nyu.edu/arpl/)<br />
Maintainer: Vivek Radhakrishnan, vr2171@nyu.edu<br />**

#### Subscribed Topics
|Name|Type|Description|
|---|---|---|
|`/hummingbird0/track/bounding_box`|geometry_msgs/PoseArray|output 2D position from Flightmare rosbag|
|`/hummingbird0/camera/rgb`|sensor_msgs/Image|RGB image|
|`/hummingbird0/imu`|sensor_msgs/Imu|IMU data|position

#### Published Topics
|Name|Type|Description|
|---|---|---|
|`/tracked_image`|sensor_msgs/Image|RGB with position, ID association labeled|
|`/tracked_pose_output`|geometry_msgs/PoseArray|position (x,y) in pixel coordinate with ID association, which is the index array|
|`/tracked_velocity_output`|geometry_msgs/PoseArray|linear velocity (x,y) in pixel coordinate with ID association, velocity in pixel space|

#### ROS Parameters
|Name|Description|
|---|---|
|`filter`|phd or jpdaf specify in the demo.launch file|
|`input_bbox_topic`|output of py_imag_proc or /hummingbird0/track/bounding_box specify in the demo.launch file|
|`input_rgb_topic`|/hires/image_raw or hummingbird0/camera/rgb specify in the demo.launch file|
|`input_imu_topic`|output of dragonfly imu or hummingbird0/imu specify in the demo.launch file|
|`num_drones`|how many drones in FOV specify in the demo.launch file|
|`camera_cx`|CX paramerter of the camera projection matrix|
|`camera_cy`|CY paramerter of the camera projection matrix|
|`camera_f`'|F of the camera projection matrix|
|`dt`|Hard coded time difference between imu frames (not used anymore)|
|`viz_detection_height`|Image height used by the detection algorithm|
|`viz_detection_width`|Image width used by the detection algorithm|
|`viz_detection_offset_x`|Output drawing offset in X axis in case of black bars|
|`viz_detection_offset_y`|Output drawing offset in Y axis in case of black bars|
|`enable_async_pdf`|Not used anymore|
|`use_generated_id`|Assigns sequential IDs to subsequent detections|
|`phd/q_pos`|Process noise for the evolution of position|
|`phd/q_vel`|Process noise for the evolution of velocity|
|`phd/r_meas`|Measurement noise|
|`phd/p_pos_init`|Initial process covariance for position|
|`phd/p_vel_init`|Initial process covariance for velocity|
|`phd/prune_weight_threshold`|Minimum threshold weight for pruning. If the weight is lower than this value, its removed from the next step|
|`phd/prune_mahalanobis_threshold`|Maximum mahalanobis distance for merging different updates|
|`phd/extract_weight_threshold`|Minimum threshold for the weight for state extraction. If the weight is lower than this value, we update the state using the preditction instead of update|
|`jpdaf/q_pos`|Process noise for the evolution of position|
|`jpdaf/q_vel`|Process noise for the evolution of velocity|
|`jpdaf/p_pos_init`|Initial process covariance for position|
|`jpdaf/p_vel_init`|Initial process covariance for velocity|
|`jpdaf/r_meas`|Measurement noise|
|`jpdaf/alpha_0_threshold`|JPDAF Alpha 0 threshold|
|`jpdaf/alpha_cam`|pixel size ratio, use 1 for normal images|
|`jpdaf/associaiton_cost`|Association cost for JPDAF|
|`jpdaf/beta_0_threshold`|JPDAF beta 0 threshold|
|`jpdaf/false_measurements_density`|JPDAF false measurement threshold|
|`jpdaf/gamma`|JPDAF gamma|
|`jpdaf/max_missed_rate`|JPDAF max missed rate|
|`jpdaf/min_acceptance_rate`|JPDAF minimim acceptance rate|
|`jpdaf/probability_detection`|JPDAF probability of detection|

## Install
The tracking filter package is dependent on Eigen and Boost, and ROS. The additional repo can be installed below:


clone the filter repository into catkin src directory and build using catkin
```
$ git clone https://github.com/arplaboratory/multi_robot_tracking.git
$ cd ..
$ catkin build
$ source devel/setup.bash
```

retrieve exp2 rosbag data from [ARPL data folder](https://drive.google.com/drive/folders/1dSBd08ocj_x8MGDS-cl1F2HeHKFADlqP?usp=sharing) after gaining access

## Running
This pacakge runs the tracking filter only -- it doesn't provide image detection. If image detection package is not available, can run with either by recorded rosbag data or by acquiring ground truth detection from simulation. Boths options are shown below. Specifiy filter rosparam in the demo.launch file to select phd or jpdaf filter. 

```
$ roslaunch multi_robot_tracking demo.launch
```

A. Testing with recorded Rosbag. </br>
Move rosbag data into corresponding directory. Modify the rostopic subscriber in demo.launch if wanting to modify using a different measured input.
```
$ roslaunch multi_robot_tracking demo.launch
-- open a new tab and navigate to rosbag directory
$ rosbag play "bag_file_name".bag --clock 
```

B. Testing with simulation. </br>
We can also utilize the Flightare simulator for photorealistic rendering to create new data. Refer to [Flightmare github page](https://github.com/arplaboratory/flightmare) for installation and setting up dependencies. This modified version of fligtmare also depends on a modified version of rotors simulator. This can be found here [Rotors Simulator github page](https://github.com/arplaboratory/rotors_simulator)<br />
Running the below code launches Flightmare simulation with Gazebo for multiple drones, multimotion creates trajectories for all drones, and phd_tracker associates each target in space.
```
$ roslaunch flightros multi_robot.launch 
-- open a new tab
$ roslaunch multi_robot_tracking demo.launch
-- open a new tab
$ rosrun multi_robot_tracking flightmare_test.py
```

C. Testing Matlab for Evaluation
Move the corresponding rosbag data to the Matlab directory. Make sure the GM_PHD_Initialisation_drones.m file points to the correct directory for rosbag data. 

