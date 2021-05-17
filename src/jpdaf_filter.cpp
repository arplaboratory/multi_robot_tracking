#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "multi_robot_tracking/JpdafFilter.h"
#include <multi_robot_tracking/detection.h>

using namespace std;


JpdafFilter::JpdafFilter()
{
  initialize_matrix();
}

void JpdafFilter::initialize_matrix()
{
  ROS_INFO("test");
}

void JpdafFilter::track()
{
  auto detections = get_detections_flightmare(Z_k);

      ROS_INFO("Detections:");
      for(int d=0; d<(int)detections.size(); d++)
      {
          cout << detections[d].getVect() << endl;
      }

}

std::vector<Detection> JpdafFilter::get_detections_flightmare(Eigen::MatrixXf& z_matrix)
{
//  ROS_WARN("Converting PoseArray to Detection format");
    std::vector<Detection> norm_det;
    for(uint i=0;  i < z_matrix.cols(); i++)
    {
      Detection one_det(z_matrix(0,i),z_matrix(1,i),1,1);
      norm_det.push_back(one_det);
    }

    return norm_det;
}
