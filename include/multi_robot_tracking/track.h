#ifndef _TRACK_H_
#define _TRACK_H_

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <multi_robot_tracking/tracker_param.h>
#include <multi_robot_tracking/kalman.h>


  class Track
  {
    public:
      Track(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params);

      void predict(float dt, Eigen::Vector3f omega);

      void setId(const int _id)
      {
        id = _id;
      }

      const int getId() const
      {
        return id;
      }
      const Eigen::Matrix2f S() const
      {
        return KF->getS();
      }

      void update(const std::vector<Detection> detections, std::vector<double> beta, double beta_0);

      void increase_lifetime()
      {
        life_time++;
      }

      void print_life_time(){ROS_INFO("lifetime: %d", life_time);}

      void has_not_been_detected()
      {
        noDetections++;
      }

      void has_been_detected()
      {
        noDetections=0;
      }

      bool isDeprecated()
      {
        return noDetections >= maxMissedRate;
      }

      bool isValidated()
      {
        return life_time >= minAcceptanceRate;
      }

      Eigen::Vector2f get_z(){return KF->get_z();}

      cv::RotatedRect get_error_ellipse(double chisquare_val);
    private:
      int id;
      int maxMissedRate;
      int minAcceptanceRate;
      std::shared_ptr<Kalman> KF;
      int noDetections;
      int life_time;
  };


#endif
