#ifndef _TRACKER_PARAM_H_
#define _TRACKER_PARAM_H_

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;

  class TrackerParam
  {
    public:
      float pd = 0.952662721893;
      float gamma = 10;
      double false_measurements_density = 0.00000003034;
      double beta_0_threshold = 0.5;
      double alpha_0_threshold = 0.95;
      int max_missed_rate = 15;
      int min_acceptance_rate = 15;
      Eigen::Matrix2f R;
      //Eigen::Matrix2f T;
      Eigen::Vector2f T;
      Eigen::Matrix4f P_0;
      int nb_drones = 3;
      float assoc_cost = 50;

      float max_update_time_rate = 0.1;

      float focal_length = 564.26;
      float alpha_cam = 1.0; //pixel size ratio
      Eigen::Vector2f principal_point;

      string gt_topic_name;
      string source_odom_name;
      std::vector<string> target_odom_names;

      string root_;
      string output_file_name_;



      TrackerParam(ros::NodeHandle nh_priv_);
      TrackerParam(){};

  };



#endif
