
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

//ros
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

//detection bbox
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

//CV
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <fstream>
#include <iostream>
#include <iomanip>

#define PI 3.14159

namespace phd_tracker {

class Node {
    public:
      Node(const ros::NodeHandle nh, const ros::NodeHandle pnh);
      
    private:
        //ros node handle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        image_transport::ImageTransport it_;


        ros::Subscriber detection_sub_;
        ros::Subscriber real_detection_sub_;
        ros::Subscriber flightmare_detection_sub_;
        ros::Subscriber image_sub_;
        ros::Subscriber flightmare_image_sub_;
        image_transport::Publisher image_pub_;
        //ros::Subscriber source_odom_sub_;
        //std::vector<ros::Subscriber> target_odom_subs_;

        ros::Time startTime,endTime,processTime;

        geometry_msgs::PoseArray Z_current_k;
        int detected_size_k;

        float last_timestamp_synchronized;
        double last_timestamp_from_rostime;
        bool first_callback = true;
        int numTargets_Jk_k_minus_1;
        int numTargets_Jk_minus_1;
        int L = 0;


        //kalman filter variables
        Eigen::MatrixXf F;
        Eigen::MatrixXf Q;
        Eigen::MatrixXf R;
        Eigen::MatrixXf K;

        //phd variables

        float prob_survival = 1.0;
        float prob_detection = 1.0;

        Eigen::MatrixXf mk_minus_1;
        Eigen::MatrixXf wk_minus_1;
        Eigen::MatrixXf Pk_minus_1;
        Eigen::MatrixXf mk_k_minus_1;
        Eigen::MatrixXf wk_k_minus_1;
        Eigen::MatrixXf Pk_k_minus_1;
        Eigen::MatrixXf P_k_k;

        Eigen::MatrixXf S;

        Eigen::MatrixXf mk;
        Eigen::MatrixXf wk;
        Eigen::MatrixXf Pk;

        Eigen::MatrixXf mk_bar;
        Eigen::MatrixXf wk_bar;
        Eigen::MatrixXf Pk_bar;

        Eigen::MatrixXf mk_bar_fixed;
        Eigen::MatrixXf wk_bar_fixed;
        Eigen::MatrixXf Pk_bar_fixed;

        Eigen::MatrixXf mk_k_minus_1_beforePrediction;

        Eigen::MatrixXf Z_k;
        Eigen::MatrixXf X_k;

        cv::Mat input_image;
        sensor_msgs::ImagePtr image_msg;



//----------------------------


        void flightmare_detectionCallback(const geometry_msgs::PoseArrayPtr& bounding_boxes);
        void imageCallback(const sensor_msgs::ImageConstPtr& img_msg);

        void imuCallback(const sensor_msgs::Imu& imu_msg);
        void imageCallback(const sensor_msgs::Image& image_msg);
        void detection_Callback(const geometry_msgs::PoseArray& pose_msg);
        void real_detectionCallback(const darknet_ros_msgs::BoundingBoxesPtr& bounding_boxes);
        void phd_track();
        void phd_predict_existing();
        void phd_construct();
        void phd_update();
        void phd_prune();
        void phd_state_extract();
        void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
        void removeColumnf(Eigen::MatrixXf& matrix, unsigned int colToRemove);
        void draw_image();

        void initialize();
        float clutter_intensity(const float X, const float Y);
        Eigen::MatrixXf left_divide(const Eigen::MatrixXf);

        
        void timer_callback(const ros::TimerEvent& event);

};

} 
