#include <multi_robot_tracking/track.h>


Track::Track(const float& x, const float& y, const float& vx, const float& vy, TrackerParam params)
{

    params.R << 2, 0, 0, 2;
    params.T << 50000, 50000;

    params.P_0 << 20, 0, 0, 0,
                   0, 20, 0, 0,
                   0, 0, 8, 0,
                   0, 0, 0, 8;

    params.principal_point << 489.72, 267.87;


    KF = std::shared_ptr<Kalman>(new Kalman(x, y, vx, vy, params));
    life_time = 0;
    noDetections = 0;
    maxMissedRate = params.max_missed_rate;
    minAcceptanceRate = params.min_acceptance_rate;
    id = -1;
}


void Track::predict(float dt, Eigen::Vector3f omega)
{
    KF->predict(dt, omega);
}


void Track::update(const std::vector<Detection> detections, std::vector<double> beta, double beta_0)
{
    KF->update(detections, beta, beta_0);
}


cv::RotatedRect Track::get_error_ellipse(double chisquare_val){

    //Get the eigenvalues and eigenvectors
    cv::Mat eigenvalues, eigenvectors;

    cv::Mat S_cv;

    eigen2cv(S(), S_cv);

    cv::eigen(S_cv, eigenvalues, eigenvectors);

    //Calculate the angle between the largest eigenvector and the x-axis
    double angle = atan2(-eigenvectors.at<float>(0,1), eigenvectors.at<float>(0,0)); // - minus sign because y axis is inverted

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
        angle += 2*M_PI;

    //Conver to degrees instead of radians
    angle = 180*angle/M_PI;

    //Calculate the size of the minor and major axes
    float halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<float>(0));
    float halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<float>(1));

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    cv::Point2f mean((int)(get_z())(0), (int)(get_z())(1));
    return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);



}
