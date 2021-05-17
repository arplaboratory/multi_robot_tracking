#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <math.h>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

//detection bbox
//#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBox.h>

//CV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <fstream>
#include <iostream>
#include <iomanip>

//jpdaf
#include <multi_robot_tracking/detection.h>


#define PI 3.14159
#define NUM_DRONES 3


class JpdafFilter
{
public:
  JpdafFilter();

  //functions
  void initialize_matrix();
  void track();
  std::vector<Detection> get_detections_flightmare(Eigen::MatrixXf& z_matrix);

  //variables
  int detected_size_k;
  bool first_callback = true;
  Eigen::MatrixXf Z_k;



private:


};

