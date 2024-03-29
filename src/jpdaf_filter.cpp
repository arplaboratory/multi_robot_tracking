#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "multi_robot_tracking/JpdafFilter.h"
#include <multi_robot_tracking/detection.h>

#include <multi_robot_tracking/track.h>
#include <multi_robot_tracking/hungarian_alg.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

JpdafFilter::JpdafFilter()
{
  initialize_matrix();
}

void JpdafFilter::initialize_matrix()
{
  //ROS_INFO("init matrix");
//    R_cam_imu << -0.9994192917454484, -0.015152697184557497, 0.03052007626234578,
//                    0.003454872095575218, -0.9361296277702554, -0.351638143365486,
//                    0.03389901393594679, -0.3513285012331849, 0.9356383601987547;

    R_cam_imu << 0 , -1, 0,
                0, 0, -1,
                1, 0, 0;
}

void JpdafFilter::track(bool called_from_detection)
{
    auto time_start = high_resolution_clock::now();
    startTime = ros::Time::now();
  geometry_msgs::PoseArray latest_pose_detection;

  if(called_from_detection)
  {
    latest_pose_detection = flightmare_bounding_boxes_msgs_buffer_.back();
  }

  else
  {
    std::vector<geometry_msgs::Pose> emptyVector_pose;
    latest_pose_detection.poses = emptyVector_pose;
  }

  auto detections = get_detections_flightmare(latest_pose_detection);

//      ROS_INFO("Detections:");
//      for(int d=0; d<(int)detections.size(); d++)
//      {
//          cout << detections[d].getVect() << endl;
//      }

  if(track_init)
  {
    ROS_WARN("Tracker not initialized");
    for (uint d=0; d<detections.size(); d++)
    {
      prev_unassoc_detections.push_back(detections[d]);
    }

    //create_tracks_test_input();//ttt

    if(called_from_detection)
    {
        last_timestamp_synchronized = latest_pose_detection.header.stamp.toSec();
        last_timestamp_from_rostime = ros::Time::now().toSec();
        last_track_from_detection = true;
        ROS_WARN("stamp: %f, now: %f", last_timestamp_synchronized, last_timestamp_from_rostime);
        ROS_INFO("Tracker initialized");
        track_init = false;
    }
  }

  else
  {
      cout << "Currently tracked tracks indexes: " << endl; for(auto tr : tracks_){cout << tr.getId() << " ";} cout << endl;
      double time_step;
      if(called_from_detection && last_track_from_detection)
      {
//          time_step = latest_pose_detection.header.stamp.toSec() - last_timestamp_synchronized; // testing new timestamp method
          time_step = ros::Time::now().toSec() - last_timestamp_from_rostime;
          ROS_WARN("dt: %f" , time_step);
      }
      else
      {
          time_step = ros::Time::now().toSec() - last_timestamp_from_rostime;
      }


      ROS_INFO("tracking called with time step %f, detection boxes nb: %d", time_step, (int)detections.size());

      if(time_step < 0)
      {
          ROS_FATAL("Negative time step! %f", time_step);// Should not happen anymore
          exit(0);
      }

      if(time_step != 0.0)//Sometimes Darknet ros publishes consecutive detections (bounding boxes and detections) with same timestamp, this happens when no new image is available in the yolo's buffer of images to perform detection
      {

          auto omega = compute_angular_velocity((double)(last_timestamp_synchronized + time_step));

          //PREDICTION
          std::vector<Eigen::Vector2f> projected_predictions;


          ROS_WARN("========= predict step ========");
          for(uint t=0; t<tracks_.size(); t++)
          {
              tracks_[t].predict(time_step, omega);
              projected_predictions.push_back(tracks_[t].get_z());

          }

          //------------
          //UPDATE

          auto assoc_mat = association_matrix(detections);
          //std::cout << "assoc_mat: " << endl << assoc_mat << endl;

          auto hypothesis_mats = generate_hypothesis_matrices(assoc_mat);
          auto hypothesis_probs = compute_probabilities_of_hypothesis_matrices(hypothesis_mats, detections);
          ROS_INFO("Nb of hypotheses: %d", (int)hypothesis_mats.size());

          /*cout << "hypothesis matrices and their respective probabilities:" << endl;
          for(uint h=0; h<hypothesis_mats.size(); h++)
          {
              cout << hypothesis_mats[h] << endl << "prob: " <<hypothesis_probs[h] << endl << endl;
          }*/

          auto betas_matrix = compute_betas_matrix(hypothesis_mats, hypothesis_probs);
//            cout << "betas_matrix: " << endl << betas_matrix << endl;

          std::vector<double> betas_0;
          for(uint t=0; t<tracks_.size(); t++)
          {
              ROS_INFO("1");
              std::vector<double> beta(betas_matrix.rows());
              ROS_INFO("2");
              double beta_0 = 1.0;
              for(int m=0; m<betas_matrix.rows(); m++)
              {
                  beta[m] = betas_matrix(m, t+1);
                  beta_0 -= betas_matrix(m, t+1);
              }
              betas_0.push_back(beta_0);
              tracks_[t].update(detections, beta, beta_0);

              cout << "tracks: " << endl << tracks_[t].get_z() << endl;
          }
          std::vector<double> alphas_0; for(int m=0; m<betas_matrix.rows(); m++){alphas_0.push_back(betas_matrix(m, 0));} // betas 0 are probabilities that track t has not been detected. Alphas 0 are probabilities that measurement m was generated by clutter noise. So beta 0 does NOT correspond to first column  of betas matrix

          manage_new_old_tracks(detections, alphas_0, betas_0, omega, time_step); //ttt

          //Update of variables for next call
          if(called_from_detection)
          {
            last_timestamp_synchronized = latest_pose_detection.header.stamp.toSec();
            last_track_from_detection = true;
          }

          else
          {
              last_timestamp_synchronized += ros::Time::now().toSec() - last_timestamp_from_rostime;
              last_track_from_detection = false;
          }

          last_timestamp_from_rostime = ros::Time::now().toSec();

          endTime = ros::Time::now();
          float planTime = (endTime - startTime).toSec();
          ROS_ERROR("total track time: %f [sec]", (endTime - startTime).toSec());
          
          auto time_end = high_resolution_clock::now();
          auto duration = duration_cast<std::chrono::microseconds>(time_end - time_start);
          ROS_ERROR_STREAM("Time taken by function: " << duration.count() << " microseconds" << endl);

//          draw_tracks_publish_image(detections, (double)(last_timestamp_synchronized + time_step), projected_predictions);
//                   publishTracks((double)(last_timestamp_synchronized + time_step));

      }

  }
  flightmare_bounding_boxes_msgs_buffer_.clear();

  debug_track_counter++;

  update_timer.start();

}

std::vector<Detection> JpdafFilter::get_detections_flightmare(const geometry_msgs::PoseArray latest_detection)
{
//  ROS_WARN("Converting PoseArray to Detection format");
    std::vector<Detection> norm_det;
    for(uint i=0;  i < latest_detection.poses.size(); i++)
    {
      Detection one_det(latest_detection.poses[i].position.x,latest_detection.poses[i].position.y,1,1);
      norm_det.push_back(one_det);
    }

    return norm_det;
}

Eigen::Vector3f JpdafFilter::compute_angular_velocity(double detection_timestamp)
{
    ROS_INFO("Imu buffer length: %d", (int)imu_buffer_.size());
    ROS_INFO("Detection timestamp: %f", detection_timestamp);

    if(!imu_buffer_ok(detection_timestamp))
    {
        Eigen::Vector3f omega(0, 0, 0);
        return omega;
    }


    int imu_buffer_index = 0;
    while(imu_buffer_index != (int)imu_buffer_.size()-1)
    {
        if((double)imu_buffer_[imu_buffer_index].header.stamp.toSec() >= detection_timestamp)
        {
            break;
        }
        imu_buffer_index++;
    }

    ROS_INFO("Selected imu buffer timestamp: %f", imu_buffer_[imu_buffer_index].header.stamp.toSec());

    Eigen::Vector3f omega_imu;
    omega_imu << imu_buffer_[imu_buffer_index].angular_velocity.x, imu_buffer_[imu_buffer_index].angular_velocity.y, imu_buffer_[imu_buffer_index].angular_velocity.z;

    Eigen::Vector3f omega_cam;
    omega_cam = R_cam_imu*omega_imu;

    cout << "omega cam: " << endl << omega_cam << endl;

    std::vector<sensor_msgs::Imu> temp;
    for(int i=imu_buffer_index; i<(int)imu_buffer_.size(); i++)
    {
        temp.push_back(imu_buffer_[i]);
    }
    imu_buffer_.clear();
    imu_buffer_ = temp;

    return omega_cam;

}

bool JpdafFilter::imu_buffer_ok(double detection_timestamp)
{
    if((int)imu_buffer_.size() > 1000)
    {
        ROS_FATAL("Imu buffer is too long, there is a problem somewhere");
        exit(1);
    }
    else if((int)imu_buffer_.size() == 0)
    {
//        ROS_ERROR("Imu buffer length is 0. Assuming no orientation change");
        return false;
    }
    else if(detection_timestamp - imu_buffer_.back().header.stamp.toSec() > 0.5)
    {
        ROS_ERROR("timestamp: %f, imu: %f", detection_timestamp, imu_buffer_.back().header.stamp.toSec() );
        ROS_ERROR("Imu buffer is running too late compaired to the detections");
//        exit(0);
        return false;
    }
    else if(detection_timestamp - imu_buffer_.front().header.stamp.toSec() < 0)
    {
        ROS_WARN("Imu buffer doesn't contain elements prior to the detection. Assuming to angular velocity");
        return false;
    }
    else
    {
        return true;
    }
}

Eigen::MatrixXf JpdafFilter::association_matrix(const std::vector<Detection> detections)
{
    Eigen::MatrixXf q(detections.size(), tracks_.size()+1);
    q.setZero();

    for(uint i=0; i<detections.size(); i++) {q(i,0)=1;} // Setting first column to 1

    for(uint i=0; i<detections.size(); i++)
    {
        for (uint j=0; j<tracks_.size(); j++)
        {
            Eigen::Vector2f measure = detections[i].getVect();
            Eigen::Vector2f prediction = tracks_[j].get_z();
            if((measure-prediction).transpose() * tracks_[j].S().inverse() * (measure-prediction) <= pow(params.gamma, 2))
            {
                q(i, j+1)=1;
            }
        }
    }
    return q;
}

std::vector<Eigen::MatrixXf> JpdafFilter::generate_hypothesis_matrices(Eigen::MatrixXf assoc_mat)
{
    std::vector<Eigen::MatrixXf> hypothesis_matrices;

    if(assoc_mat.rows() == 0)
    {
        Eigen::MatrixXf hypothesis(assoc_mat.rows(), assoc_mat.cols());
        hypothesis.setZero();
        hypothesis_matrices.push_back(hypothesis);
        return hypothesis_matrices;
    }

    std::vector<std::vector<int>> non_zero_indexes_per_row;
    for(int i=0; i<assoc_mat.rows(); i++)
    {
        non_zero_indexes_per_row.push_back(get_nonzero_indexes_row(assoc_mat.row(i)));
    }

    std::vector<int> row_iterators(assoc_mat.rows(), 0);

    bool finished = false;

    while(!finished)
    {
        //creating hypothesis matrix and pushing it if feasible
        Eigen::MatrixXf hypothesis(assoc_mat.rows(), assoc_mat.cols());
        hypothesis.setZero();
        for(int i=0; i<assoc_mat.rows(); i++)
        {
            hypothesis(i, (non_zero_indexes_per_row[i])[row_iterators[i]])=1;
        }

        Eigen::MatrixXf col_sum(1, hypothesis.cols());
        col_sum.setZero();
        for(int i=0; i < hypothesis.rows(); ++i)
        {
            col_sum += hypothesis.row(i);
        }

        bool feasible = true;
        for(int j=1;j<hypothesis.cols(); j++)
        {
            if(col_sum(0,j)>1)
            feasible = false;
        }
        if(feasible)
        {
            hypothesis_matrices.push_back(hypothesis);
        }

        //switching iterators for next hypothesis matrix
        row_iterators[0] = row_iterators[0]+1;
        for(int i=0; i<assoc_mat.rows(); i++)
        {
            if(row_iterators[i] == (int)non_zero_indexes_per_row[i].size())
            {
                if(i != assoc_mat.rows() -1)
                {
                    row_iterators[i] = 0;
                    row_iterators[i+1]++;
                }
                else
                {
                    finished = true;
                }
            }
        }
    }

    return hypothesis_matrices;
}

double JpdafFilter::probability_of_hypothesis_unnormalized(Eigen::MatrixXf hypothesis, std::vector<Detection> detections)
{
    auto tau_ = tau(hypothesis);
    auto delta_ = delta(hypothesis);
    int nb_associated_measurements = (int)tau_.sum();
    int phi = tau_.rows() - nb_associated_measurements; // nb of false measurements
    int nb_associated_tracks = (int)delta_.sum();

    std::vector<Eigen::Vector2f> y_tilds;
    std::vector<Eigen::Matrix2f> Ss;

    for(int i=0; i<hypothesis.rows(); i++)
    {
        if(tau_(i, 0) == 1)
        {
            int measurement_index = i;
            int track_index;
            auto track_indexes = get_nonzero_indexes_row(hypothesis.row(i));
            if(track_indexes.size() != 1)
                ROS_ERROR("hypothesis matrix uncorrect, multiple sources for same measure! If this happens there is an error in the code");
            track_index = track_indexes[0] - 1;
            y_tilds.push_back(detections[measurement_index].getVect() - tracks_[track_index].get_z());
            //cout << "y_tild: " << endl << detections[measurement_index].getVect() - tracks_[track_index].get_z_predict() << endl;
            Ss.push_back(tracks_[track_index].S());
        }

    }

    int M = 2;//dimensionality of observations

    double product_1 = 1;
    for(int i=0; i< nb_associated_measurements; i++)
    {
        product_1 *= (exp(-(double)(y_tilds[i].transpose()*Ss[i].inverse()*y_tilds[i])/2)) / (sqrt(pow(2*M_PI, M) * Ss[i].determinant()));

    }
    double product_2 = pow(params.pd, nb_associated_tracks);
    double product_3 = pow((1-params.pd), hypothesis.cols() -1 - nb_associated_tracks);
    double probability = pow(params.false_measurements_density, phi) * product_1 * product_2 * product_3;

    return probability;
}

Eigen::MatrixXf JpdafFilter::tau(Eigen::MatrixXf hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    Eigen::MatrixXf row_sum(hypothesis.rows(), 1);
    row_sum.setZero();
    for(int i=1; i < hypothesis.cols(); ++i)
    {
        row_sum += hypothesis.col(i);
    }
    return row_sum;
}

Eigen::MatrixXf JpdafFilter::delta(Eigen::MatrixXf hypothesis)
{
    //THIS FUNCTION ASSUMES A VALID HYPOTHESIS MATRIX, NO CHECKS ARE PERFORMED
    Eigen::MatrixXf col_sum(1, hypothesis.cols());
    col_sum.setZero();
    for(int i=0; i < hypothesis.rows(); ++i)
    {
        col_sum += hypothesis.row(i);
    }
    Eigen::MatrixXf delta(1, hypothesis.cols()-1);
    delta.setZero();
    delta = col_sum.block(0, 1, 1, (int)(hypothesis.cols()-1));
    return delta;
}

std::vector<double> JpdafFilter::compute_probabilities_of_hypothesis_matrices(std::vector<Eigen::MatrixXf> hypothesis_matrices, std::vector<Detection> detections)
{
    std::vector<double> probabilities;
    if(hypothesis_matrices[0].rows() == 0)
    {
        double prob = 1;
        probabilities.push_back(prob);
        return probabilities;
    }
    for(uint h=0; h<hypothesis_matrices.size(); h++)
    {
        auto prob = probability_of_hypothesis_unnormalized(hypothesis_matrices[h], detections);
        probabilities.push_back(prob);
    }
    //Normalization:
    double sum = 0;
    for(uint p=0; p<probabilities.size(); p++)
    {
        sum += probabilities[p];
    }
    if(sum == 0)
    {
        ROS_ERROR("sum of probabilities is 0. This may mean the parameters are uncorrect.");
        for(uint i=0; i<probabilities.size(); i++)
        {
            probabilities[i] = 1/probabilities.size();
        }
    }
    else
    {
        for(uint i=0; i<probabilities.size(); i++)
        {
            probabilities[i] /= sum;
        }
    }
    return probabilities;
}

Eigen::MatrixXf JpdafFilter::compute_betas_matrix(std::vector<Eigen::MatrixXf> hypothesis_mats, std::vector<double> hypothesis_probs)
{
    Eigen::MatrixXf betas_matrix(hypothesis_mats[0].rows(), hypothesis_mats[0].cols());
    betas_matrix.setZero();
    for(int i=0; i<(int)hypothesis_mats.size(); i++)
    {
        betas_matrix += hypothesis_probs[i]*hypothesis_mats[i];
    }
    return betas_matrix;
}

std::vector<Track> JpdafFilter::create_new_tracks(std::vector<Detection> detections, std::vector<int> unassoc_detections_idx, Eigen::Vector3f omega, double time_step)
{
    std::vector<Track> new_tracks;
    int fff = 0;

    const uint& prev_unassoc_size = prev_unassoc_detections.size();
    const uint& unassoc_size = unassoc_detections_idx.size();
    std::vector<Detection> unassoc_detections;

    for(uint i=0; i<unassoc_detections_idx.size(); i++)
    {
        unassoc_detections.push_back(detections[unassoc_detections_idx[i]]);
    }

    if(prev_unassoc_size == 0)
    {
        prev_unassoc_detections.clear(); //just to be sure
        prev_unassoc_detections = unassoc_detections;
        return new_tracks;
    }
    else if (unassoc_size == 0)
    {
        prev_unassoc_detections.clear();
        return new_tracks;
    }
    else
    {
        Eigen::MatrixXf costMat(prev_unassoc_size, unassoc_size);
        std::vector<float> costs(unassoc_size * prev_unassoc_size);
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                auto tmp = Eigen::Vector2f(unassoc_detections[j].x()-prev_unassoc_detections[i].x(), unassoc_detections[j].y()-prev_unassoc_detections[i].y());
              costs.at(i + j * prev_unassoc_size ) = tmp.norm();
                costMat(i, j) = costs.at(i + j*prev_unassoc_size);
            }
        }
        std::vector<int> assignments;
        AssignmentProblemSolver APS;
        APS.Solve(costs, prev_unassoc_size, unassoc_size, assignments, AssignmentProblemSolver::optimal);
        //returned assignments is of length previous unassigned

        const uint& assSize = assignments.size();
        Eigen::MatrixXf assigmentsBin(prev_unassoc_size, unassoc_size);
        assigmentsBin.setZero();
        for(uint i = 0; i < assSize; ++i)
        {
            if( assignments[i] != -1 && costMat(i, assignments[i]) < params.assoc_cost)//TODO: should add offset due to motion into comparison
            {
              assigmentsBin(i, assignments[i]) = 1;
            }
        }
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            for(uint j = 0; j < unassoc_size; ++j)
            {
                if(assigmentsBin(i, j))
                {
                    Eigen::MatrixXf B;
                    B = Eigen::MatrixXf(2, 3);
                    B << 0, 0, 0,
                         0, 0, 0;
                    B(0,0) = ((unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).y()-params.principal_point(1)))/params.focal_length;
                    B(0,1) = -(params.focal_length*params.alpha_cam + (unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).x()-params.principal_point(0))/(params.focal_length*params.alpha_cam));
                    B(0,2) = params.alpha_cam*(unassoc_detections.at(j).y()-params.principal_point(1));

                    B(1,0) = (params.focal_length + (unassoc_detections.at(j).y()-params.principal_point(1))*(unassoc_detections.at(j).y()-params.principal_point(1))/params.focal_length);
                    B(1,1) = -((unassoc_detections.at(j).x()-params.principal_point(0))*(unassoc_detections.at(j).y()-params.principal_point(1)))/(params.alpha_cam*params.focal_length);
                    B(1,2) = -(unassoc_detections.at(j).x()-params.principal_point(0))/params.alpha_cam;

                    auto speed_offset = B*omega;

                    const float& vx = (unassoc_detections.at(j).x()-prev_unassoc_detections.at(i).x())/time_step - speed_offset(0);
                    const float& vy = (unassoc_detections.at(j).y()-prev_unassoc_detections.at(i).y())/time_step - speed_offset(1);

                    Track tr(unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy, params);
                    new_tracks.push_back(tr);
                    ROS_INFO("created new track with position %f %f, speed %f %f", unassoc_detections.at(j).x(), unassoc_detections.at(j).y(), vx, vy);
                }
            }
        }
        Eigen::MatrixXf sumAssoc(1, unassoc_size);
        sumAssoc.setZero();
        for(uint i = 0; i < prev_unassoc_size; ++i)
        {
            sumAssoc += assigmentsBin.row(i);
        }

        prev_unassoc_detections.clear();
        for(uint i=0; i<unassoc_size; i++)
        {
            if(sumAssoc(0, i) == 0)
            {
                prev_unassoc_detections.push_back(unassoc_detections.at(i));
            }
        }
        return new_tracks;
    }
}

void JpdafFilter::manage_new_old_tracks(std::vector<Detection> detections, std::vector<double> alphas_0, std::vector<double> betas_0, Eigen::Vector3f omega, double time_step)
{
    //ROS_INFO("manage new old tracks: alphas_0 length=%d, betas_0 length=%d", (int)alphas_0.size(), (int)betas_0.size());
    for(uint i=0; i<tracks_.size(); i++)
    {
        tracks_[i].increase_lifetime();
    }

    std::vector<int> unassoc_detections_idx;

    for(uint i=0; i<alphas_0.size(); i++)
    {
        if(alphas_0[i] >= params.alpha_0_threshold)
        {
            unassoc_detections_idx.push_back((int)i);
        }
    }

    ROS_ERROR_STREAM("TEST !!");
    auto new_tracks = create_new_tracks(detections, unassoc_detections_idx, omega, time_step);
    ROS_ERROR_STREAM("TEST !!!!!!");

    for(uint j=0; j<betas_0.size(); j++)
    {
        if(betas_0[j] >= params.beta_0_threshold)
        {
            tracks_[j].has_not_been_detected();
        }
        else
        {
            tracks_[j].has_been_detected();
        }
    }

    std::vector<Track> tmp;
    for(uint t=0; t<tracks_.size(); t++)
    {
        if(!tracks_[t].isDeprecated())
        {
            tmp.push_back(tracks_[t]);
        }
        else
        {
            if(tracks_[t].getId() != -1)
            {
                lost_tracks.push_back(tracks_[t].getId());
            }
            ROS_INFO("deleted track!");
        }
    }

    for(uint t=0; t<new_tracks.size(); t++)
    {
        tmp.push_back(new_tracks[t]);
    }

    tracks_.clear();
    tracks_ = tmp;

    for(uint t=0; t<tracks_.size(); t++)
    {
        if(tracks_[t].getId() == -1 && tracks_[t].isValidated())
        {
            if(!lost_tracks.empty())
            {
                tracks_[t].setId(lost_tracks[0]);
                lost_tracks.erase(lost_tracks.begin());
            }
        }
    }
}

std::vector<int> JpdafFilter::get_nonzero_indexes_row(Eigen::MatrixXf mat)
{
    std::vector<int> nonzero_elements;
    if (mat.rows() != 1)
    {
        ROS_ERROR("get_nonzero_elements_row called, argument not row!");
        return nonzero_elements;
    }
    for (int i=0; i<mat.cols(); i++)
    {
        if(mat(0,i) != 0)
        {
            nonzero_elements.push_back(i);
        }
    }
    return nonzero_elements;
}