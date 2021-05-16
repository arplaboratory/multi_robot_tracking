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


#define PI 3.14159
#define NUM_DRONES 3

class PhdFilter
{
 public:
  PhdFilter();

  void initialize_matrix();
  void phd_track();
  void phd_predict_existing();
  void phd_construct();
  void phd_update();
  void phd_prune();
  void phd_state_extract();
  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
  void removeColumnf(Eigen::MatrixXf& matrix, unsigned int colToRemove);

  void initialize();
  float clutter_intensity(const float X, const float Y);
  Eigen::MatrixXf left_divide(const Eigen::MatrixXf);


  ros::Time startTime,endTime,processTime;

  geometry_msgs::PoseArray Z_current_k;
  int detected_size_k;

  float last_timestamp_synchronized;
  double last_timestamp_from_rostime;
  bool first_callback = true;
  int numTargets_Jk_k_minus_1;
  int numTargets_Jk_minus_1;
  int L = 0;


  //kalman filter variables
  Eigen::MatrixXf F;
  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
  Eigen::MatrixXf K;

  //phd variables

  float prob_survival = 1.0;
  float prob_detection = 1.0;

  Eigen::MatrixXf mk_minus_1;
  Eigen::MatrixXf wk_minus_1;
  Eigen::MatrixXf Pk_minus_1;
  Eigen::MatrixXf mk_k_minus_1;
  Eigen::MatrixXf wk_k_minus_1;
  Eigen::MatrixXf Pk_k_minus_1;
  Eigen::MatrixXf P_k_k;

  Eigen::MatrixXf S;

  Eigen::MatrixXf mk;
  Eigen::MatrixXf wk;
  Eigen::MatrixXf Pk;

  Eigen::MatrixXf mk_bar;
  Eigen::MatrixXf wk_bar;
  Eigen::MatrixXf Pk_bar;

  Eigen::MatrixXf mk_bar_fixed;
  Eigen::MatrixXf wk_bar_fixed;
  Eigen::MatrixXf Pk_bar_fixed;

  Eigen::MatrixXf mk_k_minus_1_beforePrediction;

  Eigen::MatrixXf Z_k;
  Eigen::MatrixXf X_k;

  cv::Mat input_image;
  sensor_msgs::ImagePtr image_msg;


 private:


};

