#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "multi_robot_tracking/JpdafFilter.h"


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
  ROS_INFO("track");
}

