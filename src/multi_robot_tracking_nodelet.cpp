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

//Simple Kalman filter class
#include <multi_robot_tracking/SimpleKalman.h>

//export and store csv
#include <iostream>
#include <fstream>

using namespace std;

bool want_export_toCSV = true;


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


    bool consensus_sort_complete = true;

    ros::Time img_timestamp;
    ros::Time prev_img_timestamp;
    ros::Time bbox_timestamp;
    ros::Time imu_timestamp;
    double previous_timestamp = 0;         //t-1
    double current_timestamp = 0;          //t
    double delta_timestamp = 0;                //dt
    double imu_time = 0;
    int detection_seq = 0;

    bool first_track_flag = false; //flag after 1st track update, use to update asynchronous prediction

    std::vector<sensor_msgs::ImageConstPtr> image_buffer_;
    std::vector<sensor_msgs::Imu> sensor_imu_buffer_;


    //filters initialize
    PhdFilter phd_filter_;
    JpdafFilter jpdaf_filter_;
    KalmanFilter kalman_filter_;

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

    //output RGB data, pose data
    cv::Mat input_image;
    cv::Mat previous_image;
    sensor_msgs::ImagePtr image_msg;
    sensor_msgs::Imu imu_;


    //3D matrices for transform
    Eigen::MatrixXf rotm_world2cam;
    Eigen::MatrixXf Hmatfiller1x4;
    Eigen::MatrixXf k_matrix3x3;
    Eigen::MatrixXf k_matrix3x3_inv;

    //init world coordinates for consensus
    Eigen::MatrixXf positions_world_coordinate;
    Eigen::MatrixXf positions_cam_coordinate;
    Eigen::MatrixXf projected_2d_initial_coord;

    Eigen::MatrixXd id_consensus;
    Eigen::MatrixXd id_array_init;


    //B matrix constants for ang velocity
    float cx, cy, f;

    float filter_dt;

    bool enable_async_pdf;

    //Detection image frame 
    int detection_height, detection_width;
    int detection_offset_x, detection_offset_y;

    //Camera frame size
    int image_height, image_width;

    //phd_filter_parameters
    float q_pos, q_vel, r_meas;
    float p_pos_init, p_vel_init;
    float phd_prune_weight_threshold;
    float phd_prune_mahalanobis_dist_threshold;
    float phd_extract_weight_threshold;

    //output csv file
    ofstream outputFile;
};


void multi_robot_tracking_Nodelet::init_matrices()
{
    ROS_INFO("init matrix for drone num: %d",num_drones);
    ROS_WARN("nodelet start init matrix... verify cam K matrix for simulation or snapdragon pro!");
    k_matrix3x3 = Eigen::MatrixXf(3,3);
    k_matrix3x3_inv = Eigen::MatrixXf(3,3);

    positions_world_coordinate =  Eigen::MatrixXf(3,num_drones);
    positions_cam_coordinate =  Eigen::MatrixXf(3,num_drones);
    projected_2d_initial_coord = Eigen::MatrixXf(2,num_drones);


    id_consensus = Eigen::MatrixXd(1,num_drones);
    id_array_init = Eigen::MatrixXd(1,num_drones);
    for(int i=0; i<num_drones; i++)
    {
        id_consensus(i) = i;
        id_array_init(i) = i;
    }
    // id_consensus << id_left, id_right ;

    

    // id_array_init(0) = id_left;
    // id_array_init(1) = id_right;


    rotm_world2cam = Eigen::MatrixXf(3,3);

    // cx = 329; //from HW
    // cy = 243;
    // f = 431;

    k_matrix3x3(0,0) = f; k_matrix3x3(0,1) = 0;   k_matrix3x3(0,2) = cx;
    k_matrix3x3(1,0) = 0;   k_matrix3x3(1,1) = f; k_matrix3x3(1,2) = cy;
    k_matrix3x3(2,0) = 0;   k_matrix3x3(2,1) = 0;   k_matrix3x3(2,2) = 1;

    k_matrix3x3_inv = k_matrix3x3.inverse();

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
        float scaleX = input_image.cols / (float)detection_width;
        float scaleY = input_image.rows / (float)detection_height;
        for(int k=0; k < jpdaf_filter_.tracks_.size(); k++)
        {
            Eigen::Vector2f temp_center;
            temp_center = jpdaf_filter_.tracks_[k].get_z();
            int scaledX = floor((temp_center[0] + detection_offset_x) * scaleX);
            int scaledY = floor((temp_center[1] + detection_offset_y) * scaleY);
            temp_center[0] = scaledX;
            temp_center[1] = scaledY;
            cv::Point2f target_center(temp_center(0), temp_center(1));
            cv::Point2f id_pos(temp_center(0),temp_center(1)+10);
            cv::circle(input_image,target_center,4, cv::Scalar(0, 210, 255), 2);
            putText(input_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5

            //draw cross
            cv::Point2f det_cross_a(temp_center(0)-5, temp_center(1)-5);
            cv::Point2f det_cross_b(temp_center(0)+5, temp_center(1)-5);
            cv::Point2f det_cross_c(temp_center(0)-5, temp_center(1)+5);
            cv::Point2f det_cross_d(temp_center(0)+5, temp_center(1)+5);
            line(input_image, det_cross_a, det_cross_d, cv::Scalar(255, 20, 150), 1, 1 );
            line(input_image, det_cross_b, det_cross_c, cv::Scalar(255, 20, 150), 1, 1 );
        }


    }

    else if(filter_to_use_.compare("phd") == 0) {

        //scale 224x224 to 640x480

        float scaleX = input_image.cols / (float)detection_width;
        float scaleY = input_image.rows / (float)detection_height;
//        ROS_INFO("drawing phd estimation");
        for(int k=0; k < phd_filter_.X_k.cols(); k++)
        {
            int scaledX = floor((phd_filter_.X_k(0,k) + detection_offset_x) * scaleX);
            int scaledY = floor((phd_filter_.X_k(2,k) + detection_offset_y) * scaleY);

            cv::Point2f target_center(scaledX,scaledY);
            cv::Point2f id_pos(scaledX,scaledY+10);
            cv::circle(input_image,target_center,6, cv::Scalar(0, 210, 255), 3);
            putText(input_image, to_string(int(id_consensus(k))), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5
        }

        //measured input
        for (int k=0; k < phd_filter_.Detections.cols(); k++)
        {

            int scaledX = floor((phd_filter_.Detections(0,k) + detection_offset_x) * scaleX);
            int scaledY = floor((phd_filter_.Detections(1,k) + detection_offset_y) * scaleY);
            float scaledW = phd_filter_.Detections(2,k) * scaleX;
            float scaledH = phd_filter_.Detections(3,k) * scaleY;


            cv::Point2f measured_center(scaledX, scaledY);
            //cv::Point2f id_pos(phd_filter_.Z_k(0,k),phd_filter_.Z_k(1,k)+10);
            cv::circle(input_image,measured_center,4, cv::Scalar(255, 0, 0), 2);
            //              putText(previous_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5
            cv::Point2f top_left(scaledX - scaledW/2, scaledY - scaledH/2);
            cv::Point2f bottom_right(scaledX + scaledW/2, scaledY + scaledH/2);
            cv::rectangle(input_image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);

        }
    }

    else if(filter_to_use_.compare("kalman") == 0) {

        //scale 224x224 to 640x480

        float scaleX = input_image.cols / (float)detection_width;
        float scaleY = input_image.rows / (float)detection_height;
//        ROS_INFO("drawing phd estimation");
        for(int k=0; k < num_drones; k++)
        {
            int scaledX = floor((kalman_filter_.X_k(0,k) + detection_offset_x) * scaleX);
            int scaledY = floor((kalman_filter_.X_k(2,k) + detection_offset_y) * scaleY);

            cv::Point2f target_center(scaledX,scaledY);
            cv::Point2f id_pos(scaledX,scaledY+10);
            cv::circle(input_image,target_center,6, cv::Scalar(0, 210, 255), 3);
            putText(input_image, to_string(int(id_consensus(k))), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5
        }

        //measured input
        for (int k=0; k < num_drones; k++)
        {

            int scaledX = floor((kalman_filter_.Detections(0,k) + detection_offset_x) * scaleX);
            int scaledY = floor((kalman_filter_.Detections(1,k) + detection_offset_y) * scaleY);
            float scaledW = kalman_filter_.Detections(2,k) * scaleX;
            float scaledH = kalman_filter_.Detections(3,k) * scaleY;


            cv::Point2f measured_center(scaledX, scaledY);
            //cv::Point2f id_pos(phd_filter_.Z_k(0,k),phd_filter_.Z_k(1,k)+10);
            cv::circle(input_image,measured_center,4, cv::Scalar(255, 0, 0), 2);
            //              putText(previous_image, to_string(k), id_pos, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cvScalar(0, 255, 0), 2, cv::LINE_AA);//size 1.5 --> 0.5
            cv::Point2f top_left(scaledX - scaledW/2, scaledY - scaledH/2);
            cv::Point2f bottom_right(scaledX + scaledW/2, scaledY + scaledH/2);
            cv::rectangle(input_image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);

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
    // cv::imwrite("/home/greend/Desktop/0.png", input_image);
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", input_image).toImageMsg();
    image_msg->header.stamp = img_timestamp;
    image_pub_.publish(image_msg);

    //  ROS_WARN("img time: %f",prev_img_timestamp.toSec());
    //  ROS_WARN("bbox time: %f",bbox_timestamp.toSec());

}


void multi_robot_tracking_Nodelet::ground_truth_Callback(const geometry_msgs::PoseArray &in_PoseArray)
{
    if(want_export_toCSV)
    {
        //only after phd track occurred
            ROS_WARN("inside callback");
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


                    ROS_ERROR("saving into csv");
        }
    }
}


/* callback for 2D image to store before publishing
 * input: RGB Image
 * output: N/A
 */
void multi_robot_tracking_Nodelet::image_Callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    //image timestamp is faster than detection result timestamp by approx 1 second. store image in buffer
    image_buffer_.push_back(img_msg);

    // ROS_INFO("img buff size: %lu",image_buffer_.size());

    //look up first image with timestamp smaller than
    for(int i = 0; i < image_buffer_.size(); i++)
    {
        // ROS_INFO("looking for timestamp less than: %f, image buff[i].stamp: %f",current_timestamp, image_buffer_[i]->header.stamp.toSec() );
        //if found image with matching detection sequence
        if(image_buffer_[i]->header.stamp.toSec() >= current_timestamp)
        {
            // ROS_INFO("detection timestamp: %f, image buff[i].stamp: %f", current_timestamp, image_buffer_[i]->header.stamp.toSec() );
            // ROS_INFO("FOUND IMG match");
            cout << "matched image buff index" << i << " Buffer Size: " << image_buffer_.size() <<endl;
            auto sync_image_ptr = image_buffer_[i];
            //store img pointer
            img_timestamp = image_buffer_[i]->header.stamp;
            cv_bridge::CvImageConstPtr im_ptr_ = cv_bridge::toCvShare(sync_image_ptr, "rgb8");
            input_image = im_ptr_->image;

            //draw image with matched image
            draw_image();

            //remove from img buffer
            // image_buffer_.erase(image_buffer_.begin() + i);

            break;
        }

    }
}

/* callback for imu to store for faster motion prediction
 * input: IMU Image
 * output: N/A
 */
void multi_robot_tracking_Nodelet::imu_Callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //use imu buffer
    
    ros::Duration timeDifIMU = imu_msg->header.stamp - imu_timestamp;
    phd_filter_.dt_imu = timeDifIMU.toSec();
    ROS_WARN("imu time: %f", phd_filter_.dt_imu);
    imu_.angular_velocity = imu_msg->angular_velocity;
    imu_.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
    imu_.header = imu_msg->header;
    imu_.linear_acceleration = imu_msg->linear_acceleration;
    imu_.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
    imu_.orientation = imu_msg->orientation;
    imu_.orientation_covariance = imu_msg->orientation_covariance;
    if(phd_filter_.first_callback == false)
    {
        phd_filter_.ang_vel_k(0) = imu_msg->angular_velocity.x;
        phd_filter_.ang_vel_k(1) = imu_msg->angular_velocity.y;
        phd_filter_.ang_vel_k(2) = imu_msg->angular_velocity.z;

        //apply rotation from imu2cam frame
        phd_filter_.ang_vel_k.block<3,1>(0,0) = rotm_world2cam *  phd_filter_.ang_vel_k;

        //asynchronous motion prediction
        if(first_track_flag)
        {
            phd_filter_.asynchronous_predict_existing();
            publish_tracks();
        }
    }
    // if(filter_to_use_.compare("kalman") == 0)
    // {
    //     kalman_filter_.ang_vel_k(0) = imu_msg->angular_velocity.x;
    //     kalman_filter_.ang_vel_k(1) = imu_msg->angular_velocity.y;
    //     kalman_filter_.ang_vel_k(2) = imu_msg->angular_velocity.z;

    //     //apply rotation from imu2cam frame
    //     kalman_filter_.ang_vel_k.block<3,1>(0,0) = rotm_world2cam *  phd_filter_.ang_vel_k;

    //     //asynchronous motion prediction
    //     if(first_track_flag)
    //     {
    //         kalman_filter_.kalmanPredict();
    //         publish_tracks();
    //     }
    // }
    imu_timestamp = imu_msg->header.stamp;
}

Eigen::MatrixXf multi_robot_tracking_Nodelet::get_B_ang_vel_matrix(float x, float y)
{
    Eigen::MatrixXf temp_B_matrix;
    temp_B_matrix = Eigen::MatrixXf::Zero(4,3);

    temp_B_matrix(0,0) = (x-cx)*(y-cy)/f;       temp_B_matrix(0,1) = -(pow((x-cx),2)/f)-f;  temp_B_matrix(0,2) = (y-cy);
    temp_B_matrix(1,0) = 0;                     temp_B_matrix(1,1) = 0;                     temp_B_matrix(1,2) = 0;
    temp_B_matrix(2,0) = f+(pow((y-cy),2))/f;   temp_B_matrix(2,1) = (x-cx)*(y-cy)/f;       temp_B_matrix(2,2) = -x+cx;
    temp_B_matrix(3,0) = 0;                     temp_B_matrix(3,1) = 0;                     temp_B_matrix(3,2) = 0;

    temp_B_matrix = temp_B_matrix * phd_filter_.dt_imu;
    return temp_B_matrix;

}


/* callback for 2D image to call phd track when using flightmare rosbag data
 * input: PoseArray
 * output: N/A
 */
void multi_robot_tracking_Nodelet::detection_Callback(const geometry_msgs::PoseArray& in_PoseArray)
{
    if(in_PoseArray.poses.size() > num_drones)
    {
        ROS_ERROR("MORE DETECTIONS THAN NO OF DRONES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        return;
    }

    //get time of detection
    bbox_timestamp = in_PoseArray.header.stamp;
    current_timestamp = bbox_timestamp.toSec();




    ROS_WARN("bbox time: %f, dt: %f, imu time: %f",current_timestamp, delta_timestamp, imu_time);

    // ROS_INFO("detected size: %lu ", in_PoseArray.poses.size() );
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
    else if(filter_to_use_.compare("phd") == 0)
    {

        if(phd_filter_.first_callback)
        {
            phd_filter_.set_num_drones(num_drones);
            phd_filter_.initialize_matrix(cx, cy, f, filter_dt);
        }


        phd_filter_.detected_size_k = in_PoseArray.poses.size();

        for(int i =0; i < phd_filter_.detected_size_k; i++)
        {
            //store Z
            // x, y, w, h
            phd_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
            phd_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;

            phd_filter_.Detections(0,i) = in_PoseArray.poses[i].position.x;
            phd_filter_.Detections(1,i) = in_PoseArray.poses[i].position.y;
            phd_filter_.Detections(2,i) = in_PoseArray.poses[i].orientation.x;
            phd_filter_.Detections(3,i) = in_PoseArray.poses[i].orientation.y;
        }
        ROS_INFO_STREAM("Num Meas: " << phd_filter_.detected_size_k << "\n");
        ROS_INFO_STREAM("Z_k_CB: " << endl << phd_filter_.Z_k << "\n");
        ROS_INFO_STREAM("WK-1: " << phd_filter_.wk << "\n");
        if(phd_filter_.first_callback)
        {
            delta_timestamp = 0.125;//0.143; //0.225
            phd_filter_.dt_cam = delta_timestamp;

            phd_filter_.initialize(q_pos, q_vel, r_meas, p_pos_init, p_vel_init,
                                phd_prune_weight_threshold,
                                phd_prune_mahalanobis_dist_threshold,
                                phd_extract_weight_threshold);
            phd_filter_.first_callback = false;

            previous_timestamp = current_timestamp;
        }

        else 
        {


            delta_timestamp = 0.143; //hard-coded for 4.5 Hz TO DO FIX
            //      delta_timestamp = current_timestamp - previous_timestamp;
            //check for data with no timestamp and thus dt = 0

            phd_filter_.dt_cam = delta_timestamp;
            previous_timestamp = current_timestamp;
            for(int i =0; i < phd_filter_.X_k.cols(); i++)
            {
                phd_filter_.B.block<4,3>(0,3*i) = get_B_ang_vel_matrix(phd_filter_.X_k(0,i),phd_filter_.X_k(2,i));
            }

            phd_filter_.phd_track();
            first_track_flag = true;

            //after tracking, store previous Z value to update velocity
            for(int i =0; i < phd_filter_.detected_size_k; i++)
            {
                //store Z
                phd_filter_.Z_k_previous(0,i) = in_PoseArray.poses[i].position.x;
                phd_filter_.Z_k_previous(1,i) = in_PoseArray.poses[i].position.y;

            }


            phd_filter_.B = Eigen::MatrixXf::Zero(4,3*num_drones);

            //update for B ang vel matrix
            //store B matrix for ang velocity
            

            // consensus_sort();

            // imu_timestamp = in_PoseArray.header.stamp;
        }
    }

    else if(filter_to_use_.compare("kalman") == 0)
    {

        if(kalman_filter_.first_callback)
        {
            kalman_filter_.setNumDrones(num_drones);
            kalman_filter_.initializeMatrix(cx, cy, f, filter_dt);
        }


        kalman_filter_.detected_size_k = in_PoseArray.poses.size();
        

        for(int i =0; i < kalman_filter_.detected_size_k; i++)
        {
            //store Z
            // x, y, w, h
            kalman_filter_.Z_k(0,i) = in_PoseArray.poses[i].position.x;
            kalman_filter_.Z_k(1,i) = in_PoseArray.poses[i].position.y;

            kalman_filter_.Detections(0,i) = in_PoseArray.poses[i].position.x;
            kalman_filter_.Detections(1,i) = in_PoseArray.poses[i].position.y;
            kalman_filter_.Detections(2,i) = in_PoseArray.poses[i].orientation.x;
            kalman_filter_.Detections(3,i) = in_PoseArray.poses[i].orientation.y;
        }
        ROS_INFO_STREAM("Num Meas: " << kalman_filter_.detected_size_k << "\n");
        ROS_INFO_STREAM("Z_k_CB: " << endl << kalman_filter_.Z_k << "\n");
        ROS_INFO_STREAM("WK-1: " << kalman_filter_.wk << "\n");
        if(kalman_filter_.first_callback)
        {
            delta_timestamp = 0.125;//0.143; //0.225
            kalman_filter_.dt_cam = delta_timestamp;

            kalman_filter_.initialize(q_pos, q_vel, r_meas, p_pos_init, p_vel_init,
                                phd_prune_weight_threshold,
                                phd_prune_mahalanobis_dist_threshold,
                                phd_extract_weight_threshold);
            kalman_filter_.first_callback = false;

            previous_timestamp = current_timestamp;
        }

        else
        {


            delta_timestamp = 0.143; //hard-coded for 4.5 Hz TO DO FIX
            //      delta_timestamp = current_timestamp - previous_timestamp;
            //check for data with no timestamp and thus dt = 0

            kalman_filter_.dt_cam = delta_timestamp;
            previous_timestamp = current_timestamp;
            for(int i =0; i < phd_filter_.X_k.cols(); i++)
            {
                kalman_filter_.B.block<4,3>(0,3*i) = get_B_ang_vel_matrix(phd_filter_.X_k(0,i),phd_filter_.X_k(2,i));
            }

            kalman_filter_.kalmanTrack();
            ROS_INFO_STREAM("Finished track");
            first_track_flag = true;

            //after tracking, store previous Z value to update velocity
            // for(int i =0; i < phd_filter_.detected_size_k; i++)
            // {
            //     //store Z
            //     kalman_filter_.Z_k_previous(0,i) = in_PoseArray.poses[i].position.x;
            //     kalman_filter_.Z_k_previous(1,i) = in_PoseArray.poses[i].position.y;

            // }


            kalman_filter_.B = Eigen::MatrixXf::Zero(4,3*num_drones);

            //update for B ang vel matrix
            //store B matrix for ang velocity
            

            // consensus_sort();

            // imu_timestamp = in_PoseArray.header.stamp;
        }
    }

    ROS_INFO_STREAM("Pub track");
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
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    image_transport::ImageTransport it(nh);

    priv_nh.param<std::string>("filter",filter_to_use_,"phd"); //store which filter to use
    priv_nh.param<std::string>("input_bbox_topic",input_bbox_topic,"/DragonPro1/snpe_ros/detections"); //input bbox topic
    priv_nh.param<std::string>("input_img_topic",input_img_topic,"DragonPro1/image_publisher/image_raw"); //input img topic
    priv_nh.param<std::string>("input_imu_topic",input_imu_topic,"/DragonPro1/imu"); //input imu topic

    priv_nh.param<int>("num_drones",num_drones,2);

    //consensus - init coordinates read in from launch file
    priv_nh.param<float>("init_pos_self_x",init_pos_self_x,0);
    priv_nh.param<float>("init_pos_self_y",init_pos_self_y,0);
    priv_nh.param<float>("init_pos_x_left",init_pos_x_left,0);
    priv_nh.param<float>("init_pos_y_left",init_pos_y_left,0);
    priv_nh.param<float>("init_pos_x_right",init_pos_x_right,0);
    priv_nh.param<float>("init_pos_y_right",init_pos_y_right,0);

    priv_nh.param<int>("id_left",id_left,0);
    priv_nh.param<int>("id_right",id_right,0);

    priv_nh.param<float>("camera_cx", cx, 0);
    priv_nh.param<float>("camera_cy", cy, 0);
    priv_nh.param<float>("camera_f", f, 0);
    priv_nh.param<float>("dt", filter_dt, 0.225);

    priv_nh.param<int>("viz_detection_height", detection_height, 168);
    priv_nh.param<int>("viz_detection_width", detection_width, 224);
    priv_nh.param<int>("viz_detection_offset_x", detection_offset_x, 0);
    priv_nh.param<int>("viz_detection_offset_y", detection_offset_y, 28);
    priv_nh.param<bool>("use_generated_id", consensus_sort_complete,0);

    priv_nh.param<float>("phd/q_pos", q_pos, 6.25);
    priv_nh.param<float>("phd/q_vel", q_vel, 12.5);
    priv_nh.param<float>("phd/p_pos_init", p_pos_init, 5.0);
    priv_nh.param<float>("phd/p_vel_init", p_vel_init, 2.0);
    priv_nh.param<float>("phd/r_meas", r_meas, 45);
    priv_nh.param<float>("phd/prune_weight_threshold", phd_prune_weight_threshold, 1e-1);
    priv_nh.param<float>("phd/prune_mahalanobis_threshold_", phd_prune_mahalanobis_dist_threshold, 4.0);
    priv_nh.param<float>("phd/extract_weight_threshold", phd_extract_weight_threshold, 5e-1);
    

    ROS_INFO_STREAM("Consensus sort during init " << consensus_sort_complete);

    if(filter_to_use_.compare("phd") == 0) //using phd
    {
        ROS_WARN("will be using: %s", filter_to_use_.c_str());
        init_matrices(); //initialize matrix for storing 3D pose
        //associate_consensus(); //determine 2d position from known init positions

    }

    else if(filter_to_use_.compare("kalman") == 0) //using kalman
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


    image_pub_ = it.advertise("tracked_image",1);
    tracked_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("tracked_pose_output",1);
    tracked_velocity_pub_ = nh.advertise<geometry_msgs::PoseArray>("tracked_velocity_output",1);

    //init export csv file
    outputFile.open("/home/marklee/rosbag/groundtruth_estimate_Asynch.csv");
    outputFile << "Time" << "," << "GND_truth_X1" << "," << "GND_truth_Y1" << "," << "GND_truth_X2" << "," << "GND_truth_Y2" << ","
               << "est_X1" << "," << "est_Y1" << ","  << "est_X2" << "," << "est_Y2" << "," <<  std::endl;

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_robot_tracking_Nodelet, nodelet::Nodelet);
