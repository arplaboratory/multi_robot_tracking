#include <Eigen/Geometry>
#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>
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
//#define NUM_DRONES 3

class PhdFilter
{
 public:
  PhdFilter();

  void initialize_matrix(float cam_cu, float cam_cv, float cam_f, float meas_dt=0.225);
  void initialize_matrix();
  void phd_track();
  void phd_predict_existing();
  void phd_construct();
  void phd_update();
  void phd_prune();
  void phd_state_extract();
  void asynchronous_predict_existing();
  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
  void removeColumnf(Eigen::MatrixXf& matrix, unsigned int colToRemove);

  void initialize();
  void set_num_drones(int num_drones);
  float clutter_intensity(const float X, const float Y);
  Eigen::MatrixXf left_divide(const Eigen::MatrixXf);
  void update_F_matrix(float input_dt);
  void update_A_matrix(float input_dt);


  ros::Time startTime,endTime,processTime;

  geometry_msgs::PoseArray Z_current_k;
  int NUM_DRONES;
  int detected_size_k;

  float last_timestamp_synchronized;
  double last_timestamp_from_rostime;
  bool first_callback = true;
  int numTargets_Jk_k_minus_1;
  int numTargets_Jk_minus_1;
  int L = 0;

  int k_iteration = 0;
  bool flag_asynch_start = false;
  bool enable_async = true;


  //kalman filter variables
  Eigen::MatrixXf F;
  Eigen::MatrixXf A;
  Eigen::MatrixXf H;
  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
  Eigen::MatrixXf K;

  //phd variables

  float prob_survival = 1.0;
  float prob_detection = 0.9;//1.0;

  float dt_cam = 0.125; //8hz
  float dt_imu = 0.01;  //100hz

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
  Eigen::MatrixXf Z_k_previous;
  Eigen::MatrixXf ang_vel_k;
  Eigen::MatrixXf X_k;
  Eigen::MatrixXf X_k_previous;

  Eigen::MatrixXf B;

  Eigen::MatrixXf Detections;

  cv::Mat input_image;
  sensor_msgs::ImagePtr image_msg;

  float cu, cv, f;
  float dt;

  const uint8_t n_state = 4;
  const uint8_t n_meas = 2;
  const uint8_t n_input = 3;

 private:


};

