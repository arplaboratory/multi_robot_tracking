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

#include <multi_robot_tracking/track.h>


#define PI 3.14159
#define NUM_DRONES 3


class JpdafFilter
{
public:
  JpdafFilter();

  //functions
  void initialize_matrix();
  void track(bool called_from_detection);
  void timer_callback(const ros::TimerEvent& event);

  ros::Time startTime,endTime,processTime;


  std::vector<Detection> get_detections_flightmare(const geometry_msgs::PoseArray latest_detection);
  Eigen::Vector3f compute_angular_velocity(double detection_timestamp);
  bool imu_buffer_ok(double detection_timestamp);
  bool image_buffer_ok(double detection_timestamp);

  void publishTracks(double detection_timestamp);

  Eigen::MatrixXf association_matrix(const std::vector<Detection> detections);

  void manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> alphas_0, std::vector<double> betas_0, Eigen::Vector3f omega, double time_step);
  std::vector<Track> create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections, Eigen::Vector3f omega, double time_step);

  Eigen::MatrixXf compute_betas_matrix(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs);

  std::vector<Eigen::MatrixXf> generate_hypothesis_matrices(Eigen::MatrixXf assoc_mat);

  std::vector<double> compute_probabilities_of_hypothesis_matrices(std::vector<Eigen::MatrixXf> hypothesis_matrices, std::vector<Detection> detections);

  double probability_of_hypothesis_unnormalized(Eigen::MatrixXf hypothesis, std::vector<Detection> detections);

  Eigen::MatrixXf tau(Eigen::MatrixXf hypothesis);//THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
  Eigen::MatrixXf delta(Eigen::MatrixXf hypothesis);    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED

  void draw_tracks_publish_image(std::vector<Detection> detections, double detection_time_stamp, std::vector<Eigen::Vector2f> projected_predictions);

  std::vector<int> get_nonzero_indexes_row(Eigen::MatrixXf mat);


  Eigen::Matrix<double, 3,3> yprToRot(const Eigen::Matrix<double,3,1>& ypr);

  void create_tracks_test_input();

  void writeToFile(int nb_detections, Eigen::Vector3f omega_cam);


  //variables
  int detected_size_k;
  bool first_callback = true;
  Eigen::MatrixXf Z_k;

  bool track_init = true;

  std::vector<Detection> prev_unassoc_detections;
  std::vector<geometry_msgs::PoseArray> flightmare_bounding_boxes_msgs_buffer_;
  std::vector<sensor_msgs::ImageConstPtr> image_buffer_;
  std::vector<sensor_msgs::Imu> imu_buffer_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher image_debug_pub_;
  ros::Publisher tracks_pub_;

  double last_timestamp_synchronized;
  double last_timestamp_from_rostime;
  bool last_track_from_detection;

  std::vector<Track> tracks_;
  std::vector<int> lost_tracks;

  TrackerParam params;

  ros::Timer update_timer;

  Eigen::Matrix3f R_cam_imu;

  int debug_track_counter;

  fstream output_file_;



private:


};

