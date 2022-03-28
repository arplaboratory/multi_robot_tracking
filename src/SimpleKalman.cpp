#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "multi_robot_tracking/SimpleKalman.h"
#include <chrono>


using namespace std;
using namespace std::chrono;

KalmanFilter::KalmanFilter()
{
    output_file.open("/home/vivek/ws/NYU/ARPL/RAL2021/data/simple_kalman_data_test.csv");
    output_file << "Mx,y,x,y,x,y,x,y,Tx,y,x,y,x,y" << endl;
}

void KalmanFilter::kalmanTrack()
{
    auto time_start = high_resolution_clock::now();
    startTime = ros::Time::now();
    k_iteration = k_iteration + 1;
    ROS_INFO("iter: %d",k_iteration);

    kalmanPredict(); // ToDo: Can be asynch ???
    auto time_endP = high_resolution_clock::now();
    auto durationP = duration_cast<std::chrono::microseconds>(time_endP - time_start);
    ROS_ERROR_STREAM("Time taken by function predict: " << durationP.count() << " microseconds" << endl);

    addIssues(0, 0.0);
    auto time_endI = high_resolution_clock::now();
    auto durationI = duration_cast<std::chrono::microseconds>(time_endI - time_endP);
    ROS_ERROR_STREAM("Time taken by function Issue: " << durationI.count() << " microseconds" << endl);

    associateMeasurement();
    auto time_endA = high_resolution_clock::now();
    auto durationA = duration_cast<std::chrono::microseconds>(time_endA - time_endI);
    ROS_ERROR_STREAM("Time taken by function Associate: " << durationA.count() << " microseconds" << endl);

    kalmanUpdate();
    auto time_endU = high_resolution_clock::now();
    auto durationU = duration_cast<std::chrono::microseconds>(time_endU - time_endA);
    ROS_ERROR_STREAM("Time taken by function Update: " << durationU.count() << " microseconds" << endl);

    kalmanExtract();
    auto time_endE = high_resolution_clock::now();
    auto durationE = duration_cast<std::chrono::microseconds>(time_endE - time_endU);
    ROS_ERROR_STREAM("Time taken by function Extract: " << durationE.count() << " microseconds" << endl);

    auto time_end = high_resolution_clock::now();
    auto duration = duration_cast<std::chrono::microseconds>(time_end - time_start);
    ROS_ERROR_STREAM("Time taken by function: " << duration.count() << " microseconds" << endl);

    if(1)
        writeToFile();


    endTime = ros::Time::now();
    ROS_WARN("end of track iteration");
    Z_k.conservativeResize(n_meas, NUM_DRONES);
}


void KalmanFilter::initializeMatrix(float cam_cu, float cam_cv, float cam_f, float meas_dt)
{
    ROS_INFO("first initialize matrix");
    //initialize
    Z_k = Eigen::MatrixXf::Zero(n_meas,NUM_DRONES);
    Z_k_previous = Eigen::MatrixXf::Zero(n_meas,NUM_DRONES);
    ang_vel_k = Eigen::MatrixXf::Zero(n_input,1);

    mk_minus_1 = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    wk_minus_1 = Eigen::MatrixXf::Zero(1,NUM_DRONES);
    Pk_minus_1 = Eigen::MatrixXf::Zero(n_state,NUM_DRONES*4);

    mk = Eigen::MatrixXf::Zero(n_state,NUM_DRONES+NUM_DRONES*NUM_DRONES);
    wk = Eigen::MatrixXf::Zero(1,NUM_DRONES+NUM_DRONES*NUM_DRONES);
    Pk = Eigen::MatrixXf::Zero(n_state,n_state*(NUM_DRONES+NUM_DRONES*NUM_DRONES) );

    mk_k_minus_1 = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    wk_k_minus_1 = Eigen::MatrixXf::Zero(1,NUM_DRONES);
    Pk_k_minus_1 = Eigen::MatrixXf::Zero(n_state,n_state*NUM_DRONES);
    P_k_k = Eigen::MatrixXf::Zero(n_state,n_state*NUM_DRONES);
    S = Eigen::MatrixXf::Zero(n_meas,n_meas*NUM_DRONES);

    F = Eigen::MatrixXf::Zero(n_state,n_state);
    A = Eigen::MatrixXf::Zero(n_state,n_state);
    H = Eigen::MatrixXf::Zero(n_meas,n_state);
    Q = Eigen::MatrixXf::Zero(n_state,n_state);
    R = Eigen::MatrixXf::Zero(n_meas, n_meas);
    K = Eigen::MatrixXf::Zero(n_state,n_meas*NUM_DRONES);

    X_k = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    X_k_previous = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);

    B = Eigen::MatrixXf::Zero(n_state,n_input*NUM_DRONES);

    Detections = Eigen::MatrixXf::Zero(4,NUM_DRONES);


    cu = cam_cu;
    cv = cam_cv;
    f = cam_f;
    dt = 0.01;
}

void KalmanFilter::initialize(float q_pos, float q_vel, float r_meas, float p_pos_init, float p_vel_init,
                        float prune_weight_threshold, float prune_mahalanobis_threshold, float extract_weight_threshold)
{
    Eigen::MatrixXf P_k_init;
    P_k_init = Eigen::MatrixXf(n_state,n_state);
    P_k_init <<
            p_pos_init, 0,          0,          0,
            0,          p_vel_init, 0,          0,
            0,          0,          p_pos_init, 0,
            0,          0,          0,          p_vel_init;
    P_k_init = P_k_init * 1;


    for(int i = 0; i < Z_k.cols(); i ++)
    {   
        //store Z into mk (x,y)
        ROS_INFO_STREAM("ZK: \n" << Z_k << endl); 
        // mk_minus_1.block(0, i, n_state, 1) = H.transpose()*Z_k.block(0,i, n_meas,1);
        mk_minus_1.block(0, i, n_state, 1) << Z_k.block(0, i, 1, 1), 0, Z_k.block(1, i, 1, 1), 0; 
        ROS_INFO_STREAM("mk_minus_1: \n" << mk_minus_1 << endl);         
        //store pre-determined weight into wk (from matlab)
        wk_minus_1(i) = .0016;
        //store pre-determined weight into Pk (from paper)
        Pk_minus_1.block(0,i*4, n_state,n_state) = P_k_init;
    }

    A << 1,dt_cam,0,0,
            0,1,0,0,
            0,0,1,dt_cam,
            0,0,0,1;
    
    //Measurement Matrix
    H << 1, 0, 0 , 0,
         0, 0, 1, 0;

    //Process noise covariance, given in Vo&Ma.
    Q << q_pos,     0,      0,          0,
         0,         q_vel,  0,          0,
         0,         0,      q_pos,      0,
         0,         0,      0,          q_vel;

    //Measurement Noise
    R << r_meas,    0,
         0,         r_meas;

}

/* 
 * Prediction step is done async. The prediction is called whenever we get an IMU message.
 * The update step is called once we get a measurement from the network
*/
void KalmanFilter::kalmanPredict()
{
    ROS_INFO("======= 0. asynch predict ======= \n");
    //update A
    updateA(dt_cam);
    Eigen::MatrixXf Bu_temp = Eigen::MatrixXf::Zero(n_state,n_meas);
    F = Eigen::MatrixXf(n_state,n_state);
    float omega_x = ang_vel_k(0);
    float omega_y = ang_vel_k(1);
    float omega_z = ang_vel_k(2);
    float pu = 0;
    float pv = 0;

    Eigen::MatrixXf P_temp;
    P_temp = Eigen::MatrixXf(n_state,n_state);
    for (int i = 0; i < NUM_DRONES; i++)
    {
        Bu_temp = B.block(0,n_input*i, n_state,n_input) * ang_vel_k;
        ROS_INFO_STREAM("Bu:\n" << Bu_temp << endl);
        mk_minus_1.block(0,i, n_state,1) = A * mk_minus_1.block(0,i, n_state,1) + Bu_temp;

        P_temp = Pk_minus_1.block(0,n_state*i, n_state,n_state);

        pu = X_k(0,i);
        pv = X_k(2,i);
        F(0,0) = (dt_imu*omega_y*(2*cu - 2*pu))/f - (dt_imu*omega_x*(cv - pv))/f + 1;      F(0,1) = dt;   F(0,2) = dt_imu*omega_z - (dt_imu*omega_x*(cu - pu))/f;                            F(0,3) = 0;
        F(1,0) = 0;                                                                        F(1,1) = 1;    F(1,2) = 0;                                                                        F(1,3) = 0;
        F(2,0) = (dt_imu*omega_y*(cv - pv))/f - dt_imu*omega_z;                            F(2,1) = 0;    F(2,2) = (dt_imu*omega_y*(cu - pu))/f - (dt_imu*omega_x*(2*cv - 2*pv))/f + 1;      F(2,3) = dt;
        F(3,0) = 0;                                                                        F(3,1) = 0;    F(3,2) = 0;                                                                        F(3,3) = 1;
        P_temp = Q + F* P_temp * F.transpose();
        Pk_minus_1.block(0,n_state*i, n_state,n_state) = P_temp;
    }

    ROS_INFO_STREAM("WK|K-1:\n" << wk_minus_1 << endl);
    ROS_INFO_STREAM("mK|K-1:\n" << mk_minus_1 << endl);
    ROS_INFO_STREAM("PK|K-1:\n" << Pk_minus_1 << endl);
}

void KalmanFilter::associateMeasurement()
{
    ROS_INFO("============ 2. Associate ============= ");
    wk = Eigen::MatrixXf::Zero(NUM_DRONES, detected_size_k);
    if(detected_size_k==0)
        return;
    Eigen::MatrixXf thisZ, meanDelta_pdf, cov_pdf, cov_pdf_inv;

    thisZ = Eigen::MatrixXf(n_meas,1);
    meanDelta_pdf = Eigen::MatrixXf(n_meas,1);
    cov_pdf = Eigen::MatrixXf(n_meas,n_meas);
    cov_pdf_inv = Eigen::MatrixXf(n_meas,n_meas);

    Eigen::MatrixXf PHt, HPHt, identity4;
    PHt = Eigen::MatrixXf(n_state,n_meas);
    HPHt = Eigen::MatrixXf(n_meas,n_meas);
    identity4 = Eigen::MatrixXf::Identity(n_meas,n_meas);

    
    Eigen::MatrixXf w_new_exponent = Eigen::MatrixXf(1,1);

    for(int i=0; i<NUM_DRONES; i++)
    {
        PHt = Pk_minus_1.block(0,n_state*i, n_state,n_state) * H.transpose();
        HPHt = H*PHt;
        cov_pdf = HPHt ;//+ R; 
        cov_pdf_inv = cov_pdf.llt().solve(identity4);
        for(int j=0; j<detected_size_k; j++)
        {
            ROS_INFO_STREAM("J: " << j);
            thisZ = Z_k.block(0,j, n_meas,1);
            ROS_INFO_STREAM("thisZ:\n " << thisZ);
            meanDelta_pdf = thisZ - H*mk_minus_1.block(0,i, n_state,1);
            ROS_INFO_STREAM("meanDelta_pdf:\n " << meanDelta_pdf);
            w_new_exponent = -0.5 * (meanDelta_pdf.transpose() * cov_pdf_inv * meanDelta_pdf).transpose() * (meanDelta_pdf.transpose() * cov_pdf_inv * meanDelta_pdf) ;
            ROS_INFO_STREAM("w_new_exponent:\n " << w_new_exponent);
            double prob = pow(2*PI, -1/2) * pow(cov_pdf.determinant(),-0.5) * exp(w_new_exponent(0,0));
            ROS_INFO_STREAM("prob:\n " << prob);
            // float euclidean_dist
            wk(i, j) = prob;
            ROS_INFO_STREAM("wk:\n " << wk);
        }
    }

    ROS_INFO_STREAM("wk - \n" << wk);
    
    Eigen::VectorXf max_val(NUM_DRONES);
    for(int i=0; i<NUM_DRONES; i++)
    {
        max_val(i) = wk.row(i).maxCoeff(&max_indices[i]);
        ROS_INFO_STREAM("Found max at index" << max_indices[i] << " with value " << max_val(i));
        Eigen::MatrixXf::Index col_max_index;
        float col_max = wk.col(max_indices[i]).maxCoeff(&col_max_index);
        if(max_val(i) < 0)
        {
            max_indices[i] = -1;
        }
        else if(col_max > max_val(i))
        {
            ROS_WARN_STREAM("Found a bad index in associate");
            ROS_INFO_STREAM("Found col max at index" << col_max_index << " with value " << col_max);
            wk(i, max_indices[i]) = -20;
            i--;
            ROS_INFO_STREAM("wk - \n" << wk);
            continue;
        }
        else
        {
            for(int j =0; j<NUM_DRONES; j++)
            {
                ROS_INFO_STREAM("i: " << max_indices[i] << " j: " << j);
                // wk(i, j) = -100.0;
                if(max_indices[i] > -1)
                    wk(j, max_indices[i]) = -100.0;
            }
        }
        ROS_INFO_STREAM("max_indices - \n" << max_indices[i]);        
        ROS_INFO_STREAM("wk - \n" << wk);
    }

    ROS_INFO_STREAM("max_val - \n" << max_val);
    
    
}

void KalmanFilter::kalmanUpdate()
{
    ROS_INFO("============ 3. Update ============= ");
    if(detected_size_k==0)
        return;
    Eigen::MatrixXf thisZ, meanDelta_pdf, cov_pdf;

    thisZ = Eigen::MatrixXf(n_meas,1);
    meanDelta_pdf = Eigen::MatrixXf(n_meas,1);
    cov_pdf = Eigen::MatrixXf(n_meas,n_meas);

    Eigen::MatrixXf PHt, HPHt, identity4, identity2, thisPk_minus_1;
    PHt = Eigen::MatrixXf(n_state,n_meas);
    HPHt = Eigen::MatrixXf(n_meas,n_meas);
    identity4 = Eigen::MatrixXf::Identity(n_state,n_state);
    identity2 = Eigen::MatrixXf::Identity(n_meas,n_meas);
    thisPk_minus_1 = Eigen::MatrixXf(n_meas,n_meas);

    for(int i=0; i<NUM_DRONES; i++)
    {
        if(max_indices[i] > -1 && max_indices[i] < detected_size_k)
        {
            thisZ = Z_k.block(0,max_indices[i], n_meas,1);
            thisPk_minus_1 = Pk_minus_1.block(0,n_state*i, n_state,n_state);
            ROS_INFO_STREAM("thisZ \n" << thisZ);
            if(thisZ(0) == 0 && thisZ(1) == 0)
            {
                mk.block(0,i, n_state, 1) =  mk_minus_1.block(0,i, n_state,1);
                Pk.block(0,n_state*i, n_state,n_state) = thisPk_minus_1;
                continue;
            }
            meanDelta_pdf = thisZ - H*mk_minus_1.block(0,i, n_state,1);
            ROS_INFO_STREAM("meanDelta_pdf \n" << meanDelta_pdf);

            PHt = thisPk_minus_1 * H.transpose();
            ROS_INFO_STREAM("PHt \n" << PHt);
            HPHt = H*PHt;
            
            S.block(0,n_meas*i, n_meas,n_meas) = HPHt + R;
            ROS_INFO_STREAM("S \n" << S);
            K.block(0,n_meas*i, n_state,n_meas) = PHt * (HPHt + R).llt().solve(identity2);
            ROS_INFO_STREAM("K \n" << K);

            mk.block(0,i, n_state, 1) =  mk_minus_1.block(0,i, n_state,1) + K.block(0,n_meas*i, n_state,n_meas)*meanDelta_pdf;
            ROS_INFO_STREAM("mk \n" << mk);
            Pk.block(0,n_state*i, n_state,n_state) = (identity4 - K.block(0,n_meas*i, n_state,n_meas)*H)*thisPk_minus_1;
            ROS_INFO_STREAM("pk \n" << Pk);
        }
        else
        {
            mk.block(0,i, n_state, 1) =  mk_minus_1.block(0,i, n_state,1);
            Pk.block(0,n_state*i, n_state,n_state) = Pk_minus_1.block(0,n_state*i, n_state,n_state);
        }
    }
}


void KalmanFilter::kalmanExtract()
{

    ROS_INFO("============ 5. extract ============= ");
    if(detected_size_k==0)
        return;
    Eigen::MatrixXf velocity, position;
    velocity = Eigen::MatrixXf(2,1);
    position = Eigen::MatrixXf(2,1);
    float gain_fine_tuned = 2.0;

    ROS_INFO_STREAM("DT Cam: " << dt_cam << "\n");
    //update state for next iterations
    X_k = mk_minus_1;
    if(k_iteration > 3)
    {
        //update state for next iterations
        mk_minus_1 = mk;
        Pk_minus_1 = Pk.cwiseAbs();
        ROS_INFO_STREAM("mK in extract: \n" << mk_minus_1);
        ROS_INFO_STREAM("X_k in extract: \n" << X_k);
        ROS_INFO_STREAM("PK in extract: \n" << Pk_minus_1);
    }
    if (k_iteration > 3)
    {
        for (int i = 0; i < NUM_DRONES; i++)
        {
            position.block<1, 1>(0, 0) = (X_k.block<1,1>(0,i) - X_k_previous.block<1,1>(0,i));
            position.block<1, 1>(1, 0) = (X_k.block<1,1>(2,i) - X_k_previous.block<1,1>(2,i));
            velocity = position/ (dt_cam*gain_fine_tuned) ;
            mk_minus_1.block<1,1>(1,i) = velocity.block<1,1>(0,0);
            mk_minus_1.block<1,1>(3,i) = velocity.block<1,1>(1,0);
            ROS_INFO_STREAM("--- position: " << endl << position << endl);
            ROS_INFO_STREAM("--- dt_cam: " << endl << dt_cam << endl);
            ROS_INFO_STREAM("--- dt: " << endl << dt << endl);
            ROS_INFO_STREAM("--- velocity: " << endl << velocity << endl);
        }

    }
    X_k_previous = X_k;
    ROS_INFO_STREAM("X_k_previous:\n" << X_k_previous);
    
    ROS_INFO_STREAM("Z_k:\n" << Z_k);
}

void KalmanFilter::updateA(float input_dt)
{
    A << 1,input_dt,0,0,
            0,1,0,0,
            0,0,1,input_dt,
            0,0,0,1;
}

void KalmanFilter::setNumDrones(int num_drones_in)
{
    NUM_DRONES = num_drones_in;
}

void KalmanFilter::addIssues(int issue_id, float value)
{
    ROS_INFO("======= 1. Add Issue ======= \n");
    if(issue_id == 1) // Gaussian noise
    {
        float measurement_covariance = 15;
        float noise_scalar = value;
        for(int i=0; i<Z_k.cols(); i++)
        {
            Z_k(0, i) += (float((rand()%100)/100.0) * measurement_covariance * noise_scalar) ;
            Z_k(1, i) += (float((rand()%100)/100.0) * measurement_covariance * noise_scalar) ;
        }
    }
    else if(issue_id == 2) // False positives
    {
        int clutter_size = int(value);
        Z_k.conservativeResize(n_meas, detected_size_k+clutter_size);
        for(int i=0; i<clutter_size; i++)
        {
            
            Z_k(0, detected_size_k+i) = int(rand()%int(cu*2)) ;
            Z_k(1, detected_size_k+i) = int(rand()%int(cv*2)) ;
        }
        detected_size_k += clutter_size;
        ROS_INFO_STREAM("FP New ZK:\n" << Z_k);
        
    }
    else if(issue_id == 3)
    {   
        removed_columns = 0;
        float detection_probability = value;
        for(int i=0; i<detected_size_k; i++)
        {
            float random_value = (rand()%1000)/float(1000.0);
            ROS_ERROR_STREAM("False Negative index: " << i << " probability: " << random_value);
            if(random_value > detection_probability)
            {
                // removeColumn(Z_k, i);
                // detected_size_k --;
                // i --;
                // removed_columns ++;
                Z_k(0, i) = 0.0;
                Z_k(1, i) = 0.0;
            }
        }
        ROS_INFO_STREAM("FN New ZK:\n" << Z_k);
    }

}

void KalmanFilter::removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void KalmanFilter::removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void KalmanFilter::writeToFile()
{
    for(int i=0; i<detected_size_k; i++)
    {
        output_file << (int)Z_k(0, i) << "," << (int)Z_k(1, i) << ",";
        // output_file << (int)Detections(0, i) << "," << (int)Detections(1, i) << ",";
    }
    // if(Detections.cols() < detected_size_k)
    // {
    //     for(int i=Detections.cols(); i<detected_size_k; i++)
    //     {
    //         output_file << (int)Z_k(0, i) << "," << (int)Z_k(1, i) << ",";
    //     }
    // }
    for(int i=0; i<NUM_DRONES; i++)
    {
        output_file << (int)X_k(0, i) << "," << (int)X_k(2, i) << ",";
        // output_file << (int)Detections(0, i) << "," << (int)Detections(1, i) << ",";
    }
    output_file << std::endl;
    // output_file << X_k(0, 0) << "," << X_k(2, 0) << "," << X_k(0, 1) << "," << X_k(2, 1) << X_k(0, 2) << "," << X_k(2, 2) << std::endl;
}