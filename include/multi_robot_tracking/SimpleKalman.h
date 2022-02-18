#include <Eigen/Geometry>
#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>
#include <math.h>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

//CV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <fstream>
#include <iostream>
#include <iomanip>


#define PI 3.14159
//#define NUM_DRONES 3

class KalmanFilter
{
 public:
  KalmanFilter();

  void initializeMatrix(float cam_cu, float cam_cv, float cam_f, float meas_dt=0.225);
  void initialize(float q_pos, float q_vel, float r_meas, float p_pos_init, float p_vel_init,
                float prune_weight_threshold, float prune_mahalanobis_threshold, float extract_weight_threshold);
  void kalmanTrack();
  void kalmanPredict();
  void kalmanUpdate();
  void kalmanExtract();
  void associateMeasurement();
  void addIssues(int issue, float val);
  
  void setNumDrones(int num_drones);
  void updateA(float input_dt);

  void writeToFile();

  void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove);
  void removeColumn(Eigen::MatrixXf& matrix, unsigned int rowToRemove);


  ros::Time startTime, endTime, processTime;

  geometry_msgs::PoseArray Z_current_k;
  int NUM_DRONES;
  int detected_size_k;

  float last_timestamp_synchronized;
  double last_timestamp_from_rostime;
  bool first_callback = true;

  int k_iteration = 0;
  int removed_columns = 0;

  //kalman filter variables
  Eigen::MatrixXf F;
  Eigen::MatrixXf A;
  Eigen::MatrixXf H;
  Eigen::MatrixXf Q;
  Eigen::MatrixXf R;
  Eigen::MatrixXf K;

  //phd variables

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

  Eigen::MatrixXf Z_k;
  Eigen::MatrixXf Z_k_previous;
  Eigen::MatrixXf ang_vel_k;
  Eigen::MatrixXf X_k;
  Eigen::MatrixXf X_k_previous;

  Eigen::MatrixXf B;

  Eigen::MatrixXf Detections;
  Eigen::MatrixXf mahalDistance;

  cv::Mat input_image;
  sensor_msgs::ImagePtr image_msg;

  float cu, cv, f;
  float dt;

  const uint8_t n_state = 4;
  const uint8_t n_meas = 2;
  const uint8_t n_input = 3;

  uint8_t num_measurements_current;
  Eigen::MatrixXf::Index max_indices[100];

  std::ofstream output_file;

 private:


};

