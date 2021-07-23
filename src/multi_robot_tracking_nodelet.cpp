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

//phd filter class
#include <multi_robot_tracking/PhdFilter.h>

//jpdaf filter class
#include <multi_robot_tracking/JpdafFilter.h>

//export and store csv
#include <iostream>
#include <fstream>

using namespace std;

bool want_export_toCSV = false;

class multi_robot_tracking_Nodelet : public nodelet::Nodelet
{
public:
    multi_robot_tracking_Nodelet(){}
    ~multi_robot_tracking_Nodelet(){}

    void onInit(); //default init of nodelet

    //callback functions
    void detection_Callback(const geometry_msgs::PoseArray& in_PoseArray); //bbox to track
    void image_Callback(const sensor_msgs::ImageConstPtr &img_msg); //rgb raw
    void imu_Callback(const sensor_msgs::ImuConstPtr &imu_msg); //rgb raw
    void ground_truth_Callback(const geometry_msgs::PoseArray& in_PoseArray); //bbox projection from ground truth

    void image_real_Callback(const sensor_msgs::ImageConstPtr &img_msg); //detected rgb
    void image_realResized_Callback(const sensor_msgs::ImageConstPtr &img_msg); //rgb resized
    void vicon_glass_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth glasses
    void vicon_drone1_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone
    void vicon_drone2_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone
    void vicon_drone5_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg); //vicon 3D ground truth drone
    Eigen::MatrixXf get_B_ang_vel_matrix(float x, float y); //return B matrix for each measurement


    //extra helper functions
    void draw_image();
    void init_matrices();
    void publish_tracks();
    void associate_consensus();
    Eigen::MatrixXf project_2d_to_3d(Eigen::MatrixXf position);
    void consensus_sort();

    std::string filter_to_use_; //choose between phd or jpdaf
    std::string input_bbox_topic; //choose between /hummingbird1/track/bounding_box or /image_processor/objects_center
    std::string input_img_topic; //choose between /hummingbird1/track/bounding_box or /image_processor/objects_center
    std::string input_imu_topic;
    int num_drones;

    float init_pos_x_left =0; //init position to read from rosparam for consensus
    float init_pos_y_left =0;
    float init_pos_x_right = 0;
    float init_pos_y_right = 0;
    float init_pos_self_x = 0;
    float init_pos_self_y = 0;
    int id_left = 0;
    int id_right = 0;


    bool consensus_sort_complete = false;

    ros::Time img_timestamp;
    ros::Time prev_img_timestamp;
    ros::Time bbox_timestamp;
    ros::Time imu_timestamp;
    double previous_timestamp = 0;         //t-1
    double current_timestamp = 0;          //t
    double delta_timestamp = 0;                //dt
    double imu_time = 0;

    bool first_track_flag = false; //flag after 1st track update, use to update asynchronous prediction

    std::vector<sensor_msgs::ImageConstPtr> image_buffer_;
    std::vector<sensor_msgs::Imu> sensor_imu_buffer_;


    //filters initialize
    PhdFilter phd_filter_;
    JpdafFilter jpdaf_filter_;

    //sub and pub
    image_transport::Publisher image_pub_;
    ros::Publisher pose_glass2drone_pub_;
    ros::Publisher pose_glass2drone_proj_pub_;
    ros::Publisher tracked_pose_pub_;
    ros::Publisher tracked_velocity_pub_;

    ros::Subscriber detection_sub_;
    ros::Subscriber image_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber groundtruth_sub_;

    ros::Subscriber detection_real_sub_;
    ros::Subscriber image_real_sub_;
    ros::Subscriber image_realResized_sub_;
    ros::Subscriber vicon_glass_sub_;
    ros::Subscriber vicon_droneA_sub_;
    ros::Subscriber vicon_droneB_sub_;
    ros::Subscriber vicon_droneC_sub_;

    //output RGB data, pose data
    cv::Mat input_image;
    cv::Mat previous_image;
    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::Imu imu_;


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
    Eigen::MatrixXf k_matrix3x3_inv;
    Eigen::MatrixXf vicon_projectedA_2Dpose3x1;
    Eigen::MatrixXf vicon_projectedB_2Dpose3x1;
    Eigen::MatrixXf vicon_projectedC_2Dpose3x1;

    Eigen::MatrixXf vicon_projected_2DposeArray;

    //init world coordinates for consensus
    Eigen::MatrixXf positions_world_coordinate;
    Eigen::MatrixXf positions_cam_coordinate;
    Eigen::MatrixXf projected_2d_initial_coord;

    Eigen::MatrixXd id_consensus;
    Eigen::MatrixXd id_array_init;


    //B matrix constants for ang velocity
    float cx, cy, f;

    //output csv file
    ofstream outputFile;




};


void multi_robot_tracking_Nodelet::init_matrices()
{
    ROS_INFO("init matrix for drone num: %d",num_drones);
    ROS_WARN("nodelet start init matrix... verify cam K matrix for simulation or snapdragon pro!");
    droneA_3Dpose4x1 = Eigen::MatrixXf(4,1);
    droneB_3Dpose4x1 = Eigen::MatrixXf(4,1);
    droneC_3Dpose4x1 = Eigen::MatrixXf(4,1);
    glasses_3Dpose4x4 = Eigen::MatrixXf(4,4);
    glasses_3Dpose3x4 = Eigen::MatrixXf(3,4);
    k_matrix3x3 = Eigen::MatrixXf(3,3);
    k_matrix3x3_inv = Eigen::MatrixXf(3,3);
    vicon_projectedA_2Dpose3x1 = Eigen::MatrixXf(3,1);
    vicon_projectedB_2Dpose3x1 = Eigen::MatrixXf(3,1);
    vicon_projectedC_2Dpose3x1 = Eigen::MatrixXf(3,1);

    vicon_projected_2DposeArray = Eigen::MatrixXf(3,num_drones);

    positions_world_coordinate =  Eigen::MatrixXf(3,num_drones);
    positions_cam_coordinate =  Eigen::MatrixXf(3,num_drones);
    projected_2d_initial_coord = Eigen::MatrixXf(2,num_drones);


    id_consensus = Eigen::MatrixXd(1,num_drones);
    id_array_init = Eigen::MatrixXd(1,num_drones);

    //id_array_init(0) = id_left;
    //id_array_init(1) = id_right;


    Hmatfiller1x4 = Eigen::MatrixXf(1,4);
    rotm_world2cam = Eigen::MatrixXf(3,3);
    rotm_glassesOffset = Eigen::MatrixXf(3,3);

    /* [559 0 490] from tobii glass maxim's masters thesis.
   * [0 558 273]
   * [0   0   1]
   */

    /* [431 0 329] from snapdragon_pro board2.
   * [0 431 243]
   * [0   0   1]
   */

    /* [120 0 180] from flightmare simulation
   * [0 120 120]
   * [0   0   1]
   */

    cx = 329; //from flightmare simulation
    cy = 243;
    f = 431;



    k_matrix3x3(0,0) = f; k_matrix3x3(0,1) = 0;   k_matrix3x3(0,2) = cx;
    k_matrix3x3(1,0) = 0;   k_matrix3x3(1,1) = f; k_matrix3x3(1,2) = cy;
    k_matrix3x3(2,0) = 0;   k_matrix3x3(2,1) = 0;   k_matrix3x3(2,2) = 1;

    k_matrix3x3_inv = k_matrix3x3.inverse();

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

    if(filter_to_use_.compare("jpdaf") == 0)
    {
        //          ROS_INFO("drawing jpdaf estimation");
        for(int k=0; k < jpdaf_filter_.tracks_.size(); k++)
        {
            Eigen::Vector2f temp_center;
            temp_center = jpdaf_filter_.tracks_[k].get_z();

            cv::Point2f target_center(temp_center(0), temp_center(1));
            cv::Point2f id_pos(temp_center(0),temp_center(1)+10);
            cv::circle(previous_image,target_center,4, cv::Scalar(0, 210, 255), 2);
            putText(previous_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5

            //draw cross
            cv::Point2f det_cross_a(temp_center(0)-5, temp_center(1)-5);
            cv::Point2f det_cross_b(temp_center(0)+5, temp_center(1)-5);
            cv::Point2f det_cross_c(temp_center(0)-5, temp_center(1)+5);
            cv::Point2f det_cross_d(temp_center(0)+5, temp_center(1)+5);
            line(previous_image, det_cross_a, det_cross_d, cv::Scalar(255, 20, 150), 1, 1 );
            line(previous_image, det_cross_b, det_cross_c, cv::Scalar(255, 20, 150), 1, 1 );
        }


    }

    else {
//        ROS_INFO("drawing phd estimation");
        for(int k=0; k < phd_filter_.X_k.cols(); k++)
        {
            cv::Point2f target_center(phd_filter_.X_k(0,k),phd_filter_.X_k(1,k));
            cv::Point2f id_pos(phd_filter_.X_k(0,k),phd_filter_.X_k(1,k)+10);
            cv::circle(input_image,target_center,4, cv::Scalar(0, 210, 255), 2);
            putText(input_image, to_string(int(id_consensus(k))), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5
        }

        //measured input
        for (int k=0; k < phd_filter_.Z_k.cols(); k++)
        {
            cv::Point2f measured_center(phd_filter_.Z_k(0,k),phd_filter_.Z_k(1,k));
            //cv::Point2f id_pos(phd_filter_.Z_k(0,k),phd_filter_.Z_k(1,k)+10);
            cv::circle(input_image,measured_center,2, cv::Scalar(255, 0, 0), 1);
            //              putText(previous_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5

        }
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
    image_msg->header.stamp = prev_img_timestamp;
    image_pub_.publish(image_msg);

    //  ROS_WARN("img time: %f",prev_img_timestamp.toSec());
    //  ROS_WARN("bbox time: %f",bbox_timestamp.toSec());

}


void multi_robot_tracking_Nodelet::ground_truth_Callback(const geometry_msgs::PoseArray &in_PoseArray)
{
    if(want_export_toCSV)
    {
        //only after phd track occurred
        //    ROS_WARN("inside callback");
        if(first_track_flag)
        {

            Eigen::MatrixXf temp_groundtruth, temp_estimation;
            temp_groundtruth = Eigen::MatrixXf::Zero(2,num_drones);
            temp_estimation = Eigen::MatrixXf::Zero(2,num_drones);

            //store ground truth value
            for(int i =0; i < in_PoseArray.poses.size(); i++)
            {
                //store Z
                temp_groundtruth(0,i) = in_PoseArray.poses[i].position.x;
                temp_groundtruth(1,i) = in_PoseArray.poses[i].position.y;

            }

            //store X_k value
            for(int i =0; i < phd_filter_.detected_size_k; i++)
            {
                //store Z
                temp_estimation(0,i) = phd_filter_.X_k(0,i);
                temp_estimation(1,i) = phd_filter_.X_k(1,i);

            }

            //store into csv

            outputFile << ros::Time::now().toSec() << "," <<  temp_groundtruth(0,0) << "," << temp_groundtruth(1,0)<< "," <<
                          temp_groundtruth(0,1) << "," << temp_groundtruth(1,1) << "," << temp_estimation(0,0)  << "," <<
                          temp_estimation(1,0)  << "," << temp_estimation(0,1) << "," << temp_estimation(1,1) << "," << endl;


            //        ROS_ERROR("saving into csv");
        }
    }



}


/* callback for 2D image to store before publishing
 * input: RGB Image
 * output: N/A
 */
void multi_robot_tracking_Nodelet::image_Callback(const sensor_msgs::ImageConstPtr &img_msg)
{


    img_timestamp = img_msg->header.stamp;
    cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(img_msg, "rgb8");
    input_image = im_ptr_->image;


    draw_image();



    prev_img_timestamp = img_msg->header.stamp;
    cv_bridge::CvImageConstPtr prev_im_ptr_ = cv_bridge::toCvShare(img_msg, "rgb8");
    previous_image = prev_im_ptr_->image;
}

/* callback for imu to store for faster motion prediction
 * input: IMU Image
 * output: N/A
 */
void multi_robot_tracking_Nodelet::imu_Callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
//    ROS_INFO("imu cb");
    sensor_imu_buffer_.push_back(*imu_msg);

    if(sensor_imu_buffer_.size() > 5)
    {
        sensor_imu_buffer_.erase(sensor_imu_buffer_.begin());
    }

    //use imu buffer
    sensor_msgs::Imu prevImu = sensor_imu_buffer_[0];
    imu_timestamp = prevImu.header.stamp;
    imu_time = imu_timestamp.toSec();
    //ROS_WARN("bbox time: %f, dt: %f, imu time: %f",current_timestamp, delta_timestamp, imu_time);

    if(phd_filter_.first_callback == false)
    {
        phd_filter_.ang_vel_k(0) = prevImu.angular_velocity.x;
        phd_filter_.ang_vel_k(1) = prevImu.angular_velocity.y;
        phd_filter_.ang_vel_k(2) = prevImu.angular_velocity.z;

        //apply rotation from imu2cam frame
        phd_filter_.ang_vel_k.block<3,1>(0,0) = rotm_world2cam *  phd_filter_.ang_vel_k * phd_filter_.dt_imu;

        //asynchronous motion prediction
        if(first_track_flag)
        {
//            ROS_INFO("imu cb asych function start");
            phd_filter_.asynchronous_predict_existing();
            publish_tracks();
        }

    }

//    ROS_INFO("end of imu cb");

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

    //  ROS_WARN("--odom time:%d",odom_msg->header.stamp.sec );
    //  syncTime = odom_msg->header.stamp;

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
    for(int i =0; i< num_drones; i++)
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

Eigen::MatrixXf multi_robot_tracking_Nodelet::get_B_ang_vel_matrix(float x, float y)
{
    Eigen::MatrixXf temp_B_matrix;
    temp_B_matrix = Eigen::MatrixXf::Zero(4,3);

    temp_B_matrix(0,0) = (x-cx)*(y-cy)/f;       temp_B_matrix(0,1) = -(pow((x-cx),2)/f)-f;  temp_B_matrix(0,2) = (y-cy);
    temp_B_matrix(1,0) = f+(pow((y-cy),2))/f;   temp_B_matrix(1,1) = (x-cx)*(y-cy)/f;       temp_B_matrix(1,2) = -x+cx;
    temp_B_matrix(2,0) = 0;                     temp_B_matrix(2,1) = 0;                     temp_B_matrix(2,2) = 0;
    temp_B_matrix(3,0) = 0;                     temp_B_matrix(3,1) = 0;                     temp_B_matrix(3,2) = 0;

    temp_B_matrix = temp_B_matrix * phd_filter_.dt_imu;
    return temp_B_matrix;

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
    //get time of detection
    bbox_timestamp = in_PoseArray.header.stamp;
    current_timestamp = bbox_timestamp.toSec();

    ROS_WARN("bbox time: %f, dt: %f, imu time: %f",current_timestamp, delta_timestamp, imu_time);

    ROS_INFO("detected size: %lu ", in_PoseArray.poses.size() );
    jpdaf_filter_.last_timestamp_synchronized = in_PoseArray.header.stamp.toSec();

    //store Z

    //========= use jpdaf filter ===========
    if(filter_to_use_.compare("jpdaf") == 0)
    {

        jpdaf_filter_.detected_size_k = in_PoseArray.poses.size();

        //store Z
        jpdaf_filter_.flightmare_bounding_boxes_msgs_buffer_.push_back(in_PoseArray);
        //store imu
        jpdaf_filter_.imu_buffer_.push_back(imu_);

        //    jpdaf_filter_.Z_k = Eigen::MatrixXf::Zero(4,jpdaf_filter_.detected_size_k);

        //    for(int i =0; i < jpdaf_filter_.detected_size_k; i++)
        //    {
        //      jpdaf_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
        //      jpdaf_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;
        //    }

        jpdaf_filter_.track(true);
    }

    //========= use phd filter ===========
    else
    {

        if(phd_filter_.first_callback)
        {
            phd_filter_.set_num_drones(num_drones);
            phd_filter_.initialize_matrix();
        }


        phd_filter_.detected_size_k = in_PoseArray.poses.size();
        phd_filter_.Z_k = Eigen::MatrixXf::Zero(4,phd_filter_.detected_size_k);

        phd_filter_.B = Eigen::MatrixXf::Zero(4,3*phd_filter_.detected_size_k);

        for(int i =0; i < phd_filter_.detected_size_k; i++)
        {
            //store Z
            phd_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
            phd_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;

            //store B matrix for ang velocity
            phd_filter_.B.block<4,3>(0,3*i) = get_B_ang_vel_matrix(phd_filter_.Z_k(0,i),phd_filter_.Z_k(1,i));

        }

        cout << "Z_k_CB: " << endl << phd_filter_.Z_k << endl;

        if(phd_filter_.first_callback)
        {
            delta_timestamp = 0.125;
            phd_filter_.dt_cam = delta_timestamp;

            phd_filter_.initialize();
            phd_filter_.first_callback = false;

            previous_timestamp = current_timestamp;
        }

        else
        {


            delta_timestamp = 0.125; //hard-coded for 8 Hz TO DO FIX
            //      delta_timestamp = current_timestamp - previous_timestamp;
            //check for data with no timestamp and thus dt = 0

            phd_filter_.dt_cam = delta_timestamp;
            previous_timestamp = current_timestamp;

            phd_filter_.phd_track();
            first_track_flag = true;

            //after tracking, store previous Z value to update velocity
            for(int i =0; i < phd_filter_.detected_size_k; i++)
            {
                //store Z
                phd_filter_.Z_k_previous(0,i) = in_PoseArray.poses[i].position.x;
                phd_filter_.Z_k_previous(1,i) = in_PoseArray.poses[i].position.y;


            }

            //consensus_sort();

        }
    }

    publish_tracks();
}

/* given known initial projected coordinated, and Left/Right ID
 * sort the id_consensus according to the estimated target
 */
void multi_robot_tracking_Nodelet::consensus_sort()
{
    if(!consensus_sort_complete)
    {
        float euclidian_distance = 0;
        float min_distance = 1000000;
        int min_index = 0;
        //get init proj

        //get measurement

        //compare euclidian distance for each target with init projection
        for (int z = 0; z < phd_filter_.X_k.cols(); z++)
        {
            min_index = 0;
            min_distance = 100000;

            for(int index = 0; index < num_drones; index++)
            {
                cout << "X_k: " << phd_filter_.X_k(0,z) << "projX: " << projected_2d_initial_coord(0,index) << endl;
                euclidian_distance = abs(phd_filter_.X_k(0,z) - projected_2d_initial_coord(0,index)) + abs(phd_filter_.X_k(1,z) - projected_2d_initial_coord(1,index)) ;
                //store min index
                if(euclidian_distance < min_distance)
                {
                    min_distance = euclidian_distance;
                    min_index = index;
                }
            }

            id_consensus(z) = int(id_array_init(min_index));

        }

        //sort ID accordingly
        consensus_sort_complete = true;
        cout << " **************** consensus ID: " << id_consensus  << "**************** " << endl;
    }

}


/*
 *
 */
void multi_robot_tracking_Nodelet::associate_consensus()
{
    //ROS_WARN("inside consensus func");

    //get delta init positions in world coordinate
    Eigen::MatrixXf delta_left_to_self, delta_right_to_self;
    Eigen::MatrixXf projected_2d_padding;
    delta_left_to_self = Eigen::MatrixXf::Zero(3,1);
    delta_right_to_self = Eigen::MatrixXf::Zero(3,1);
    projected_2d_padding = Eigen::MatrixXf::Zero(3,num_drones);

    //delta_left_to_self(0) = init_pos_x_left - init_pos_self_x;
    //delta_left_to_self(1) = init_pos_y_left - init_pos_self_y;
    delta_left_to_self(0) = 3.4641;
    delta_left_to_self(1) = 2;
    delta_left_to_self(2) = 0;

    //    delta_right_to_self(0) = init_pos_x_right - init_pos_self_x;
    //    delta_right_to_self(1) = init_pos_y_right - init_pos_self_y;
    delta_right_to_self(0) = 3.4641;
    delta_right_to_self(1) = -2;
    delta_right_to_self(2) = 0;

    cout << "deltaL: " << endl << delta_left_to_self << endl;
    cout << "deltaR: " << endl << delta_right_to_self << endl;

    positions_world_coordinate.block<3,1>(0,0) = delta_left_to_self;
    positions_world_coordinate.block<3,1>(0,1) = delta_right_to_self;

    //get delta init positions in camera coordinate
    positions_cam_coordinate.block<3,1>(0,0) = rotm_world2cam * positions_world_coordinate.block<3,1>(0,0) ;
    positions_cam_coordinate.block<3,1>(0,1) = rotm_world2cam * positions_world_coordinate.block<3,1>(0,1) ;

    cout << "positions_cam_coordinate: " << endl << positions_cam_coordinate << endl;


    //project into 2D space
    for(int i =0; i < num_drones;i++)
    {
        projected_2d_padding.block<3,1>(0,i) = k_matrix3x3 * positions_cam_coordinate.block<3,1>(0,i);
        projected_2d_padding.block<3,1>(0,i) = projected_2d_padding.block<3,1>(0,i) / projected_2d_padding(2,i);
        projected_2d_initial_coord.block<2,1>(0,i) = projected_2d_padding.block<2,1>(0,i);
    }

    cout << "projected_2d_initial_coord: " << endl << projected_2d_initial_coord << endl;


}


void multi_robot_tracking_Nodelet::publish_tracks()
{
//    ROS_INFO("publish tracks");

    geometry_msgs::PoseArray tracked_output_pose, tracked_velocity_pose;
    geometry_msgs::Pose temp_pose, temp_velocity;

    if(filter_to_use_.compare("jpdaf") == 0)
    {
        //TO DO fill in publishing for jpdaf as well
    }

    else
    {

        //store estimated PHD X_k into tracked output
        for(int i =0; i < phd_filter_.X_k.cols(); i++) {

            temp_pose.position.x = phd_filter_.X_k(0,i);
            temp_pose.position.y = phd_filter_.X_k(1,i);
            tracked_output_pose.poses.push_back(temp_pose);

            temp_velocity.position.x = phd_filter_.X_k(2,i);
            temp_velocity.position.y = phd_filter_.X_k(3,i);
            tracked_velocity_pose.poses.push_back(temp_velocity);
        }

    }

    //publish output (stored as either jpdaf or phd)
    tracked_output_pose.header.stamp = bbox_timestamp;
    tracked_velocity_pose.header.stamp = bbox_timestamp;
    tracked_pose_pub_.publish(tracked_output_pose);
    tracked_velocity_pub_.publish(tracked_velocity_pose);

    //empty tracked output
    while(!tracked_output_pose.poses.empty()) {
        tracked_output_pose.poses.pop_back();
    }

    while(!tracked_velocity_pose.poses.empty()) {
        tracked_velocity_pose.poses.pop_back();
    }

//    ROS_INFO("end publish tracks");

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
    priv_nh.param<std::string>("input_bbox_topic",input_bbox_topic,"/DragonPro3/snpe_ros/detections"); //input bbox topic
    priv_nh.param<std::string>("input_img_topic",input_img_topic,"DragonPro3/image_publisher/image_raw"); //input img topic
    priv_nh.param<std::string>("input_imu_topic",input_imu_topic,"/DragonPro3/imu"); //input imu topic

    priv_nh.param<int>("num_drones",num_drones,1);

    //consensus - init coordinates read in from launch file
    priv_nh.param<float>("init_pos_self_x",init_pos_self_x,0);
    priv_nh.param<float>("init_pos_self_y",init_pos_self_y,0);
    priv_nh.param<float>("init_pos_x_left",init_pos_x_left,0);
    priv_nh.param<float>("init_pos_y_left",init_pos_y_left,0);
    priv_nh.param<float>("init_pos_x_right",init_pos_x_right,0);
    priv_nh.param<float>("init_pos_y_right",init_pos_y_right,0);

    priv_nh.param<int>("id_left",id_left,0);
    priv_nh.param<int>("id_right",id_right,0);




    if(filter_to_use_.compare("phd") == 0) //using phd
    {
        ROS_WARN("will be using: %s", filter_to_use_.c_str());
        init_matrices(); //initialize matrix for storing 3D pose
        //associate_consensus(); //determine 2d position from known init positions

    }

    else if (filter_to_use_.compare("jpdaf") == 0) {
        ROS_WARN("will be using: %s", filter_to_use_.c_str());
    }

    else {
        ROS_ERROR("wrong filter param input");
        return;
    }

    //bbox subscription of PoseArray Type
    detection_sub_ = priv_nh.subscribe(input_bbox_topic, 10, &multi_robot_tracking_Nodelet::detection_Callback, this);
    //img subscription
    image_sub_ = priv_nh.subscribe(input_img_topic, 10, &multi_robot_tracking_Nodelet::image_Callback, this);
    //imu subscription
    imu_sub_ = priv_nh.subscribe(input_imu_topic, 10, &multi_robot_tracking_Nodelet::imu_Callback, this);
    //groundtruth bbox subscription
    groundtruth_sub_ = priv_nh.subscribe("/hummingbird0/ground_truth/bounding_box", 10, &multi_robot_tracking_Nodelet::ground_truth_Callback, this);


    //  vicon_glass_sub_ = priv_nh.subscribe("/vicon/TobiiGlasses/odom", 10, &multi_robot_tracking_Nodelet::vicon_glass_Callback, this);
    //  vicon_droneA_sub_ = priv_nh.subscribe("/vicon/DragonFly1/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone1_Callback, this);
    //  vicon_droneB_sub_ = priv_nh.subscribe("/vicon/DragonFly2/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone2_Callback, this);
    //  vicon_droneC_sub_ = priv_nh.subscribe("/vicon/DragonFly5/odom", 10, &multi_robot_tracking_Nodelet::vicon_drone5_Callback, this);

    image_pub_ = it.advertise("tracked_image",1);
    //  pose_glass2drone_pub_ = priv_nh.advertise<nav_msgs::Odometry>("/test_detected_pose",1);
    //  pose_glass2drone_proj_pub_ = priv_nh.advertise<nav_msgs::Odometry>("/test_projected_pose",1);

    tracked_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tracked_pose_output",1);
    tracked_velocity_pub_ = nh.advertise<geometry_msgs::PoseArray>("tracked_velocity_output",1);

    //init export csv file
    outputFile.open("/home/marklee/rosbag/groundtruth_estimate_Asynch.csv");
    outputFile << "Time" << "," << "GND_truth_X1" << "," << "GND_truth_Y1" << "," << "GND_truth_X2" << "," << "GND_truth_Y2" << ","
               << "est_X1" << "," << "est_Y1" << ","  << "est_X2" << "," << "est_Y2" << "," <<  std::endl;

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_robot_tracking_Nodelet, nodelet::Nodelet);
