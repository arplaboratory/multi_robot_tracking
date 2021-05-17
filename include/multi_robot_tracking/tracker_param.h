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
      float pd;
      float gamma;
      double false_measurements_density;
      double beta_0_threshold;
      double alpha_0_threshold;
      int max_missed_rate;
      int min_acceptance_rate;
      Eigen::Matrix2f R;
      //Eigen::Matrix2f T;
      Eigen::Vector2f T;
      Eigen::Matrix4f P_0;
      int nb_drones;
      float assoc_cost;

      float max_update_time_rate;

      float focal_length;
      float alpha_cam; //pixel size ratio
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
