#ifndef _KALMAN_H_
#define _KALMAN_H_

#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <multi_robot_tracking/tracker_param.h>
#include <multi_robot_tracking/detection.h>

  class Kalman
  {
    public:
      Kalman(const float& px, const float& py, const float& vx, const float& vy, TrackerParam params);
      void predict(const float dt, const Eigen::Vector3f omega);
      void update(const std::vector< Detection> detections, const std::vector<double> beta, double beta_0); //Added by Max
      inline const Eigen::Matrix2f getS() const
      {
          return S;
      }
      Eigen::Vector2f get_z(){return z;}
    private:
      Eigen::MatrixXf C;
      Eigen::MatrixXf B;
      Eigen::Matrix2f R; //Proces measurement Covariance matrix
      Eigen::Vector2f T; //Proces measurement Covariance vector (accelerations)
      Eigen::Matrix2f S;
      Eigen::MatrixXf K; //Gain
      Eigen::Matrix4f P; //Covariance Matrix predicted error
      Eigen::Vector4f x;
      Eigen::Vector2f z;

      float f;
      float alpha;
      Eigen::Vector2f c; //principal point
  };


#endif
