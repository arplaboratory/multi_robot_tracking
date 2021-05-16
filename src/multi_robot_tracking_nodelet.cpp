#include <std_msgs/Bool.h>
#include <Eigen/Geometry>

//cv image
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

//ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//detection bbox
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

//phd filter class
#include <multi_robot_tracking/PhdFilter.h>

//jpdaf filter class
#include <multi_robot_tracking/JpdafFilter.h>

using namespace std;

class multi_robot_tracking_Nodelet : public nodelet::Nodelet
{
 public:
  multi_robot_tracking_Nodelet(){}
  ~multi_robot_tracking_Nodelet(){}

  void onInit(); //default init of nodelet

  //callback functions
  void detection_Callback(const geometry_msgs::PoseArray& in_PoseArray); //bbox to track
  void image_Callback(const sensor_msgs::ImageConstPtr &img_msg); //rgb raw
  void detection_real_Callback(const darknet_ros_msgs::BoundingBoxes& in_PoseArray); //bbox to track
  void image_real_Callback(const sensor_msgs::ImageConstPtr &img_msg); //detected rgb
  void image_realResized_Callback(const sensor_msgs::ImageConstPtr &img_msg); //rgb resized
  void vicon_glass_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth glasses
  void vicon_drone1_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone
  void vicon_drone2_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone
  void vicon_drone5_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone

  //extra helper functions
  void draw_image();
  void init_matrices();

  std::string filter_to_use_; //choose between phd or jpdaf

  //filters initialize
  PhdFilter phd_filter_;
  JpdafFilter jpdaf_filter_;

  //sub and pub
  image_transport::Publisher image_pub_;
  ros::Publisher pose_glass2drone_pub_;
  ros::Publisher pose_glass2drone_proj_pub_;

  ros::Subscriber detection_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber detection_real_sub_;
  ros::Subscriber image_real_sub_;
  ros::Subscriber image_realResized_sub_;
  ros::Subscriber vicon_glass_sub_;
  ros::Subscriber vicon_droneA_sub_;
  ros::Subscriber vicon_droneB_sub_;
  ros::Subscriber vicon_droneC_sub_;

  //output RGB data
  cv::Mat input_image;
  sensor_msgs::ImagePtr image_msg;

  //3D matrices for transform
  Eigen::MatrixXf droneA_3Dpose4x1;
  Eigen::MatrixXf droneB_3Dpose4x1;
  Eigen::MatrixXf droneC_3Dpose4x1;
  Eigen::MatrixXf glasses_3Dpose4x4;
  Eigen::MatrixXf glasses_3Dpose3x4;
  Eigen::MatrixXf rotm_world2cam;
  Eigen::MatrixXf rotm_glassesOffset;
  Eigen::MatrixXf Hmatfiller1x4;
  Eigen::MatrixXf k_matrix3x3;
  Eigen::MatrixXf vicon_projectedA_2Dpose3x1;
  Eigen::MatrixXf vicon_projectedB_2Dpose3x1;
  Eigen::MatrixXf vicon_projectedC_2Dpose3x1;

  Eigen::MatrixXf vicon_projected_2DposeArray;

  ros::Time syncTime;


};


void multi_robot_tracking_Nodelet::init_matrices()
{
//  ROS_WARN("this should come before cb");
  droneA_3Dpose4x1 = Eigen::MatrixXf(4,1);
  droneB_3Dpose4x1 = Eigen::MatrixXf(4,1);
  droneC_3Dpose4x1 = Eigen::MatrixXf(4,1);
  glasses_3Dpose4x4 = Eigen::MatrixXf(4,4);
  glasses_3Dpose3x4 = Eigen::MatrixXf(3,4);
  k_matrix3x3 = Eigen::MatrixXf(3,3);
  vicon_projectedA_2Dpose3x1 = Eigen::MatrixXf(3,1);
  vicon_projectedB_2Dpose3x1 = Eigen::MatrixXf(3,1);
  vicon_projectedC_2Dpose3x1 = Eigen::MatrixXf(3,1);
  vicon_projected_2DposeArray = Eigen::MatrixXf(3,NUM_DRONES);

  Hmatfiller1x4 = Eigen::MatrixXf(1,4);
  rotm_world2cam = Eigen::MatrixXf(3,3);
  rotm_glassesOffset = Eigen::MatrixXf(3,3);

  /* [559 0 490] from maxim's masters thesis.
   * [0 558 273]
   * [0   0   1]
   */

  k_matrix3x3(0,0) = 559; k_matrix3x3(0,1) = 0;   k_matrix3x3(0,2) = 480;
  k_matrix3x3(1,0) = 0;   k_matrix3x3(1,1) = 558; k_matrix3x3(1,2) = 270;
  k_matrix3x3(2,0) = 0;   k_matrix3x3(2,1) = 0;   k_matrix3x3(2,2) = 1;

  // [ 0 0 0 1 ]
  Hmatfiller1x4(0,0) = 0;  Hmatfiller1x4(0,1) = 0; Hmatfiller1x4(0,2) = 0; Hmatfiller1x4(0,3) = 1;

  /* [0 -1  0]
   * [0  0 -1]
   * [1  0  0]
   */
  rotm_world2cam(0,0) = 0;   rotm_world2cam(0,1) =-1;   rotm_world2cam(0,2) =  0;
  rotm_world2cam(1,0) = 0;   rotm_world2cam(1,1) = 0;   rotm_world2cam(1,2) = -1;
  rotm_world2cam(2,0) = 1;   rotm_world2cam(2,1) = 0;   rotm_world2cam(2,2) =  0;





}

/* use tracking data to draw onto 2D image
 * input: N/A
 * output: 2D image with tracking ID
 */
void multi_robot_tracking_Nodelet::draw_image()
{
//  ROS_INFO("drawing estimation");
  for(int k=0; k < phd_filter_.X_k.cols(); k++)
  {
    cv::Point2f target_center(phd_filter_.X_k(0,k),phd_filter_.X_k(1,k));
    cv::Point2f id_pos(phd_filter_.X_k(0,k),phd_filter_.X_k(1,k)+10);
    cv::circle(input_image,target_center,4, cv::Scalar(0, 210, 255), 2);
    putText(input_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, CV_AA);//size 1.5 --> 0.5
  }

//  ROS_INFO("drawing ground truth");
//  for(int k=0; k < vicon_projected_2DposeArray.cols(); k++)
//  {
//    cv::Point2f target_center(vicon_projected_2DposeArray(0,k),vicon_projected_2DposeArray(1,k));
//    cv::Point2f id_pos(vicon_projected_2DposeArray(0,k),vicon_projected_2DposeArray(1,k)+10);
//    cv::circle(input_image,target_center,4, cv::Scalar(0, 255, 0), 2);
//    putText(input_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, CV_AA);//size 1.5 --> 0.5
//  }

  image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", input_image).toImageMsg();
  image_msg->header.stamp = syncTime;
  image_pub_.publish(image_msg);

}

/* callback for 2D image to store before publishing
 * input: RGB Image
 * output: N/A
 */
void multi_robot_tracking_Nodelet::image_Callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//  ROS_INFO("image cb");
  cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(img_msg, "rgb8");
  input_image = im_ptr_->image;

}

void multi_robot_tracking_Nodelet::image_real_Callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//  ROS_INFO("image cb with HxW: %d,%d", img_msg->height, img_msg->width);
  cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(img_msg, "rgb8");
  input_image = im_ptr_->image;

//  ROS_WARN("time:%f",img_msg->header.stamp.toSec() );

  //
  draw_image();
}

void multi_robot_tracking_Nodelet::image_realResized_Callback(const sensor_msgs::ImageConstPtr &img_msg)
{
//  ROS_WARN("time:%f",img_msg->header.stamp.sec );
//  ROS_INFO("image cb with HxW: %d,%d", img_msg->height, img_msg->width);
  cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(img_msg, "rgb8");
  input_image = im_ptr_->image;

}

/* callback for 3D Vicon ground truth
 * input: odom msg
 * output: N/A
 */
void multi_robot_tracking_Nodelet::vicon_glass_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{

  ROS_WARN("--odom time:%d",odom_msg->header.stamp.sec );
  syncTime = odom_msg->header.stamp;

//  ROS_INFO("glass cb");
  //translation
  glasses_3Dpose4x4(0,3) = odom_msg->pose.pose.position.x;
  glasses_3Dpose4x4(1,3) = odom_msg->pose.pose.position.y;
  glasses_3Dpose4x4(2,3) = odom_msg->pose.pose.position.z;

  //rotation
  Eigen::Quaternionf q;
  q.x() = odom_msg->pose.pose.orientation.x;
  q.y() = odom_msg->pose.pose.orientation.y;
  q.z() = odom_msg->pose.pose.orientation.z;
  q.w() = odom_msg->pose.pose.orientation.w;

  Eigen::Matrix3f rotation_mat = q.normalized().toRotationMatrix();
  glasses_3Dpose4x4.block<3,3>(0,0) = rotation_mat.block<3,3>(0,0);

  //pad 4x[1:4] with [ 0 0 0 1 ]
  glasses_3Dpose4x4.block<1,4>(3,0) = Hmatfiller1x4.block<1,4>(0,0);

//  cout << "glass_wrt_world: " << endl << glasses_3Dpose4x4 << endl;
//  cout << "drone_wrt_world: " << endl << droneA_3Dpose4x1 << endl;

  Eigen::MatrixXf droneA_3Dpose3x1_glass,droneA_3Dpose4x1_glass;
  droneA_3Dpose4x1_glass = Eigen::MatrixXf(4,1);
  droneA_3Dpose3x1_glass = Eigen::MatrixXf(3,1);

  Eigen::MatrixXf droneB_3Dpose3x1_glass,droneB_3Dpose4x1_glass;
  droneB_3Dpose4x1_glass = Eigen::MatrixXf(4,1);
  droneB_3Dpose3x1_glass = Eigen::MatrixXf(3,1);

  Eigen::MatrixXf droneC_3Dpose3x1_glass,droneC_3Dpose4x1_glass;
  droneC_3Dpose4x1_glass = Eigen::MatrixXf(4,1);
  droneC_3Dpose3x1_glass = Eigen::MatrixXf(3,1);

  //Drone position w.r.t to Glass
  droneA_3Dpose4x1_glass = glasses_3Dpose4x4.inverse() * droneA_3Dpose4x1;
  droneA_3Dpose3x1_glass = droneA_3Dpose4x1_glass.block<3,1>(0,0);

  droneB_3Dpose4x1_glass = glasses_3Dpose4x4.inverse() * droneB_3Dpose4x1;
  droneB_3Dpose3x1_glass = droneB_3Dpose4x1_glass.block<3,1>(0,0);

  droneC_3Dpose4x1_glass = glasses_3Dpose4x4.inverse() * droneC_3Dpose4x1;
  droneC_3Dpose3x1_glass = droneC_3Dpose4x1_glass.block<3,1>(0,0);
//  cout << "drone_wrt_glass3x1: " << endl << droneA_3Dpose3x1_glass << endl;

  /* lamda [x y 1]' = k * [R t] * [Xw Yw Zw 1]'
   * 3D to 2D projective equation
   */
  vicon_projectedA_2Dpose3x1 = k_matrix3x3 * rotm_world2cam * droneA_3Dpose3x1_glass;
  float scaling_factor = vicon_projectedA_2Dpose3x1(2);
  vicon_projectedA_2Dpose3x1 = vicon_projectedA_2Dpose3x1 / scaling_factor;

  vicon_projectedB_2Dpose3x1 = k_matrix3x3 * rotm_world2cam * droneB_3Dpose3x1_glass;
  scaling_factor = vicon_projectedB_2Dpose3x1(2);
  vicon_projectedB_2Dpose3x1 = vicon_projectedB_2Dpose3x1 / scaling_factor;

  vicon_projectedC_2Dpose3x1 = k_matrix3x3 * rotm_world2cam * droneC_3Dpose3x1_glass;
  scaling_factor = vicon_projectedC_2Dpose3x1(2);
  vicon_projectedC_2Dpose3x1 = vicon_projectedC_2Dpose3x1 / scaling_factor;

//  cout << "VICON 2Dprojected: (" << vicon_projectedA_2Dpose3x1(0) << "," << vicon_projectedA_2Dpose3x1(1) << ") " <<
//        "(" << vicon_projectedB_2Dpose3x1(0) << "," << vicon_projectedB_2Dpose3x1(1) << ") "
//        "(" << vicon_projectedC_2Dpose3x1(0) << "," << vicon_projectedC_2Dpose3x1(1) << ") " << endl;

  //store into vicon projection array
  vicon_projected_2DposeArray.block<3,1>(0,0) = vicon_projectedA_2Dpose3x1;
  vicon_projected_2DposeArray.block<3,1>(0,1) = vicon_projectedB_2Dpose3x1;
  vicon_projected_2DposeArray.block<3,1>(0,2) = vicon_projectedC_2Dpose3x1;

  //offset Y for vicon to glass error
  for(int i =0; i< NUM_DRONES; i++)
  {
    vicon_projected_2DposeArray(1,i) = vicon_projected_2DposeArray(1,i) - 180; //1/3 of the height offset?
  }

  //test projection correlation direction by plotting
  /*
  nav_msgs::Odometry test_pose, test_proj_pose;
  test_pose.header.stamp = odom_msg->header.stamp;
  test_pose.pose.pose.position.x = droneA_3Dpose3x1_glass(0);
  test_pose.pose.pose.position.y = droneA_3Dpose3x1_glass(1);
  test_pose.pose.pose.position.z = droneA_3Dpose3x1_glass(2);

  test_proj_pose.header.stamp = odom_msg->header.stamp;
  test_proj_pose.pose.pose.position.x = vicon_projected_2Dpose3x1(0);
  test_proj_pose.pose.pose.position.y = vicon_projected_2Dpose3x1(1);
  test_proj_pose.pose.pose.position.z = vicon_projected_2Dpose3x1(2);

  pose_glass2drone_pub_.publish(test_pose);
  pose_glass2drone_proj_pub_.publish(test_proj_pose);
  */

}

/* callback for 3D Vicon ground truth
 * input: odom msg
 * output: N/A [Xw Yw Zw 1]'
 */
void multi_robot_tracking_Nodelet::vicon_drone1_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
//  ROS_INFO("drone cb");
  droneA_3Dpose4x1(0) = odom_msg->pose.pose.position.x;
  droneA_3Dpose4x1(1) = odom_msg->pose.pose.position.y;
  droneA_3Dpose4x1(2) = odom_msg->pose.pose.position.z;
  droneA_3Dpose4x1(3) = 1;

//  cout << "dronematrix: " << endl << droneA_3Dpose4x1 << endl;
}

/* callback for 3D Vicon ground truth
 * input: odom msg
 * output: N/A
 */
void multi_robot_tracking_Nodelet::vicon_drone2_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
//  ROS_INFO("drone cb");
  droneB_3Dpose4x1(0) = odom_msg->pose.pose.position.x;
  droneB_3Dpose4x1(1) = odom_msg->pose.pose.position.y;
  droneB_3Dpose4x1(2) = odom_msg->pose.pose.position.z;
  droneB_3Dpose4x1(3) = 1;



}

/* callback for 3D Vicon ground truth
 * input: odom msg
 * output: N/A
 */
void multi_robot_tracking_Nodelet::vicon_drone5_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
//  ROS_INFO("drone cb");
  droneC_3Dpose4x1(0) = odom_msg->pose.pose.position.x;
  droneC_3Dpose4x1(1) = odom_msg->pose.pose.position.y;
  droneC_3Dpose4x1(2) = odom_msg->pose.pose.position.z;
  droneC_3Dpose4x1(3) = 1;


}

/* callback for 2D image to call phd track when using flightmare rosbag data
 * input: PoseArray
 * output: N/A
 */
void multi_robot_tracking_Nodelet::detection_Callback(const geometry_msgs::PoseArray& in_PoseArray)
{
  ROS_INFO("detected size: %lu ", in_PoseArray.poses.size() );
  //store Z

  //========= use jpdaf filter ===========
  if(filter_to_use_.compare("jpdaf") == 0)
  {

    jpdaf_filter_.detected_size_k = in_PoseArray.poses.size();
    jpdaf_filter_.Z_k = Eigen::MatrixXf::Zero(4,jpdaf_filter_.detected_size_k);


    for(int i =0; i < jpdaf_filter_.detected_size_k; i++)
    {
      jpdaf_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
      jpdaf_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;
    }

    if(jpdaf_filter_.first_callback)
    {

      jpdaf_filter_.initialize_matrix();
      jpdaf_filter_.first_callback = false;

    }

    else {

      jpdaf_filter_.track();

    }


  }

  //========= use phd filter ===========
  else
  {
    phd_filter_.detected_size_k = in_PoseArray.poses.size();
    phd_filter_.Z_k = Eigen::MatrixXf::Zero(4,phd_filter_.detected_size_k);

    for(int i =0; i < phd_filter_.detected_size_k; i++)
    {
      phd_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
      phd_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;
    }

    cout << "Z_k: " << endl << phd_filter_.Z_k << endl;


    if(phd_filter_.first_callback)
    {

      phd_filter_.initialize();
      phd_filter_.first_callback = false;

    }

    else {
      phd_filter_.phd_track();
      draw_image();
    }
  }
}

/* callback for 2D image to call phd track when using flightmare rosbag data
 * input: PoseArray
 * output: N/A
 */
void multi_robot_tracking_Nodelet::detection_real_Callback(const darknet_ros_msgs::BoundingBoxes& in_PoseArray)
{
  ROS_INFO("detected size: %lu ", in_PoseArray.bounding_boxes.size() );
  //store Z
  phd_filter_.detected_size_k = in_PoseArray.bounding_boxes.size();
  phd_filter_.Z_k = Eigen::MatrixXf::Zero(4,phd_filter_.detected_size_k);

  for(int i =0; i < phd_filter_.detected_size_k; i++)
  {
    phd_filter_.Z_k(0,i) = int((in_PoseArray.bounding_boxes[i].xmin+in_PoseArray.bounding_boxes[i].xmax)/2);
    phd_filter_.Z_k(1,i) =  int((in_PoseArray.bounding_boxes[i].ymin+in_PoseArray.bounding_boxes[i].ymax)/2);
  }

  cout << "Z_k: " << endl << phd_filter_.Z_k << endl;


  if(phd_filter_.first_callback)
  {

    phd_filter_.initialize();
    phd_filter_.first_callback = false;

  }

  else {
    phd_filter_.phd_track();
    draw_image();

  }



}
/* Nodelet init function to handle subscribe/publish
 * input: N/A
 * output: N/A
 */
void multi_robot_tracking_Nodelet::onInit(void)
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  image_transport::ImageTransport it(nh);

  priv_nh.param<std::string>("filter",filter_to_use_,"phd"); //store which filter to use


  if(filter_to_use_.compare("phd") == 0) //using phd
  {
    ROS_WARN("will be using: %s", filter_to_use_.c_str());
    //init_matrices(); //initialize matrix for storing 3D pose

  }

  else if (filter_to_use_.compare("jpdaf") == 0) {
    ROS_WARN("will be using: %s", filter_to_use_.c_str());
  }

  else {
    ROS_ERROR("wrong filter param input");
    return;
  }


  //When using simulation data
  detection_sub_ = priv_nh.subscribe("/hummingbird1/track/bounding_box", 10, &multi_robot_tracking_Nodelet::detection_Callback, this);
//  image_sub_ = priv_nh.subscribe("/hummingbird1/camera/rgb", 10, &multi_robot_tracking_Nodelet::image_Callback, this);

  //When using real data
//  detection_real_sub_ = priv_nh.subscribe("/darknet_ros/bounding_boxes", 10, &multi_robot_tracking_Nodelet::detection_real_Callback, this);
  image_real_sub_ = priv_nh.subscribe("/darknet_ros/detection_image", 10, &multi_robot_tracking_Nodelet::image_real_Callback, this);
//  image_realResized_sub_ = priv_nh.subscribe("/gaze/image_resized", 10, &multi_robot_tracking_Nodelet::image_realResized_Callback, this);

//  vicon_glass_sub_ = priv_nh.subscribe("/vicon/TobiiGlasses/odom", 10, &multi_robot_tracking_Nodelet::vicon_glass_Callback, this);
//  vicon_droneA_sub_ = priv_nh.subscribe("/vicon/DragonFly1/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone1_Callback, this);
//  vicon_droneB_sub_ = priv_nh.subscribe("/vicon/DragonFly2/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone2_Callback, this);
//  vicon_droneC_sub_ = priv_nh.subscribe("/vicon/DragonFly5/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone5_Callback, this);

  image_pub_ = it.advertise("tracked_image",1);
//  pose_glass2drone_pub_ = priv_nh.advertise<nav_msgs::Odometry>("/test_detected_pose",1);
//  pose_glass2drone_proj_pub_ = priv_nh.advertise<nav_msgs::Odometry>("/test_projected_pose",1);

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_robot_tracking_Nodelet, nodelet::Nodelet);
