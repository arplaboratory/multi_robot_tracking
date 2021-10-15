#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include "multi_robot_tracking/PhdFilter.h"


using namespace std;

PhdFilter::PhdFilter()

{

}

void PhdFilter::phd_track()
{
    startTime = ros::Time::now();
    k_iteration = k_iteration + 1;
    ROS_DEBUG("iter: %d",k_iteration);

    //construct
    phd_construct();
    //update
    phd_update();
    //prune
    phd_prune();
    //state extraction
    phd_state_extract();

    endTime = ros::Time::now();
    ROS_WARN("end of track iteration");

}


void PhdFilter::initialize_matrix(float cam_cu, float cam_cv, float cam_f, float meas_dt)
{
    ROS_DEBUG("first initialize matrix");
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

    mk_bar = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    wk_bar = Eigen::MatrixXf::Zero(1,NUM_DRONES);
    Pk_bar = Eigen::MatrixXf::Zero(n_state,n_state*NUM_DRONES);

    mk_bar_fixed = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    wk_bar_fixed = Eigen::MatrixXf::Zero(1,NUM_DRONES);
    Pk_bar_fixed = Eigen::MatrixXf::Zero(n_state,n_state*NUM_DRONES);

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

    mk_k_minus_1_beforePrediction = Eigen::MatrixXf::Zero(n_state,NUM_DRONES);
    Detections = Eigen::MatrixXf::Zero(4,NUM_DRONES);


    cu = cam_cu;
    cv = cam_cv;
    f = cam_f;
    dt = 0.01;
}

void PhdFilter::set_num_drones(int num_drones_in)
{
    NUM_DRONES = num_drones_in;
}

void PhdFilter::initialize()
{
    Eigen::MatrixXf P_k_init;
    P_k_init = Eigen::MatrixXf(n_state,n_state);
    P_k_init <<
            10,0,0,0,
            0,10,0,0,
            0,0,5,0,
            0,0,0,5;


    for(int i = 0; i < Z_k.cols(); i ++)
    {   
        //store Z into mk (x,y)
        ROS_DEBUG_STREAM("ZK: \n" << Z_k << endl); 
        // mk_minus_1.block(0, i, n_state, 1) = H.transpose()*Z_k.block(0,i, n_meas,1);
        mk_minus_1.block(0, i, n_state, 1) << Z_k.block(0, i, 1, 1), 0, Z_k.block(1, i, 1, 1), 0; 
        ROS_DEBUG_STREAM("mk_minus_1: \n" << mk_minus_1 << endl);         
        //store pre-determined weight into wk (from matlab)
        wk_minus_1(i) = .0016;
        //store pre-determined weight into Pk (from paper)
        Pk_minus_1.block(0,i*4, n_state,n_state) = P_k_init;
    }

    A << 1,dt_imu,0,0,
            0,1,0,0,
            0,0,1,dt_imu,
            0,0,0,1;
    
    //Measurement Matrix
    H << 1, 0, 0 , 0,
         0, 0, 1, 0;

    //Process noise covariance, given in Vo&Ma.
    Q << 6.25,      0,      0,          0,
         0,         12.5,   0,          0,
         0,         0,      6.25,       0,
         0,         0,      0,          12.5;
    Q = Q*0.5;

    //Measurement Noise
    R << 45,   0,
         0,     45;

    numTargets_Jk_minus_1 = NUM_DRONES;
}

/* 
 * Prediction step is done async. The prediction is called whenever we get an IMU message.
 * The update step is called once we get a measurement from the network
*/
void PhdFilter::asynchronous_predict_existing()
{
    ROS_DEBUG("======= 0. asynch predict ======= \n");
    //update A
    update_A_matrix(dt_imu);

    wk_minus_1 = prob_survival * wk_minus_1;
    Eigen::MatrixXf Bu_temp = Eigen::MatrixXf::Zero(n_state,n_meas);
    F = Eigen::MatrixXf(n_state,n_state);

    float omega_x = ang_vel_k(0);
    float omega_y = ang_vel_k(1);
    float omega_z = ang_vel_k(2);
    float pu = 0;
    float pv = 0;

    Eigen::MatrixXf P_temp;
    P_temp = Eigen::MatrixXf(n_state,n_state);

    ROS_DEBUG("size mk-1: %lu, size B: %lu",mk_minus_1.cols(), B.cols());
    ROS_DEBUG_STREAM("A:\n" << A << endl);
    for (int i = 0; i < mk_minus_1.cols(); i++)
    {
        ROS_DEBUG("iteration: %d", i);
        ROS_DEBUG_STREAM("B:\n" << B.block(0,n_input*i, n_state,n_input)); 
        Bu_temp = B.block(0,n_input*i, n_state,n_input) * ang_vel_k; //
        ROS_DEBUG_STREAM("Bu:\n" << Bu_temp); 
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

    X_k = mk_minus_1;
    wk_k_minus_1 = wk_minus_1;
    mk_k_minus_1 = mk_minus_1;
    Pk_k_minus_1 = Pk_minus_1;
    numTargets_Jk_k_minus_1 = numTargets_Jk_minus_1;

    ROS_DEBUG_STREAM("WK|K-1:\n" << wk_k_minus_1 << endl);
    ROS_DEBUG_STREAM("mK|K-1:\n" << mk_k_minus_1 << endl);
    ROS_DEBUG_STREAM("PK|K-1:\n" << Pk_k_minus_1 << endl);
}


void PhdFilter::phd_construct()
{
    ROS_DEBUG("======= 2. construct ======= \n");
    wk_k_minus_1 = wk_minus_1;
    mk_k_minus_1 = mk_minus_1;
    Pk_k_minus_1 = Pk_minus_1;
    numTargets_Jk_k_minus_1 = numTargets_Jk_minus_1;

    Eigen::MatrixXf PHt, HPHt, identity4;
    PHt = Eigen::MatrixXf(n_state,n_meas);
    HPHt = Eigen::MatrixXf(n_meas,n_meas);

    identity4 = Eigen::MatrixXf::Identity(4,4);

    for(int j = 0; j < numTargets_Jk_k_minus_1; j ++)
    {
        PHt = Pk_k_minus_1.block(0,n_state*j, n_state,n_state) * H.transpose();
        HPHt = H*PHt;
        K.block(0,n_meas*j, n_state,n_meas) = PHt * (HPHt + R).inverse();
        S.block(0,n_meas*j, n_meas,n_meas) = HPHt + R;
        Eigen::MatrixXf t1 = (identity4 - K.block(0,n_meas*j, n_state,n_meas)*H)*Pk_k_minus_1.block(0,n_state*j, n_state,n_state);
        P_k_k.block(0,n_state*j, n_state,n_state) = t1;
    }
    ROS_DEBUG_STREAM("P_k_k: " << endl << P_k_k << endl);
    ROS_DEBUG_STREAM("K: " << endl << K << endl);
}

void PhdFilter::phd_update()
{
    ROS_DEBUG("======= 3. update ======= \n");

    //1. set up matrix size
    wk = Eigen::MatrixXf::Zero(1,numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1);
    mk = Eigen::MatrixXf::Zero(n_state,numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1);
    Pk = Eigen::MatrixXf::Zero(n_state, n_state * (numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1));


    Eigen::MatrixXf thisZ, meanDelta_pdf, cov_pdf;
    Eigen::MatrixXf w_new, w_new_exponent;
    

    thisZ = Eigen::MatrixXf(n_meas,1);
    meanDelta_pdf = Eigen::MatrixXf(n_meas,1);
    cov_pdf = Eigen::MatrixXf(n_meas,n_meas);
    w_new = Eigen::MatrixXf(1,1);
    w_new_exponent = Eigen::MatrixXf(1,1);

    int index = 0;
    L = 0;

    for (int i = 0; i < numTargets_Jk_k_minus_1; i++ )
    {
        wk(i) = (1-prob_detection) * wk_k_minus_1(i);
        mk.block(0,i, n_state,1)  = mk_k_minus_1.block(0,i, n_state,1);
        Pk.block(0,n_state*i, n_state,n_state) = Pk_k_minus_1.block(0,n_state*i, n_state,n_state);
    }

    for (int z = 0; z < detected_size_k; z++)
    {
        L = L+1;

        for (int j = 0; j < numTargets_Jk_k_minus_1; j++)
        {
            thisZ = Z_k.block(0,z, n_meas,1);

            index = (L) * numTargets_Jk_k_minus_1 + j; //3~11
            ROS_DEBUG("Index: %d, L: %d\n", index, L);
            //update weight (multivar prob distr)
            meanDelta_pdf = thisZ - H*mk_k_minus_1.block(0,j, n_state,1);
            ROS_DEBUG_STREAM("Mean Delta PDF: " << meanDelta_pdf << endl);

            cov_pdf = S.block(0,n_meas*j, n_meas,n_meas);
            ROS_DEBUG_STREAM("Cov PDF: " << cov_pdf << endl);
            w_new_exponent = -0.5 * (meanDelta_pdf.transpose() * cov_pdf.inverse() * meanDelta_pdf) * (meanDelta_pdf.transpose() * cov_pdf.inverse() * meanDelta_pdf) ;
            ROS_DEBUG_STREAM("W_new_exponent: " << w_new_exponent << endl);

            double w_val = wk_k_minus_1(j) * pow(2*PI, -1/2) * pow(cov_pdf.determinant(),-0.5) * exp(w_new_exponent(0,0));
            ROS_DEBUG_STREAM("w_val: " << w_val << endl);
            wk(index)= w_val;

            //update mean
            mk.block(0,index, n_state,1) = mk_k_minus_1.block(0,j, n_state,1);
            mk.block(0,index, n_state, 1) =  mk_k_minus_1.block(0,j, n_state,1) + K.block(0,n_meas*j, n_state,n_meas)*meanDelta_pdf;
            //update cov
            Pk.block(0,n_state*index, n_state,n_state) = P_k_k.block(0,n_state*j, n_state,n_state);

//            ROS_INFO("wk: %f",wk(index));
//            ROS_DEBUG_STREAM << "mk: "<< mk <<  endl;

        }


        ROS_DEBUG_STREAM("mk(pre): " << endl << mk << endl);
        ROS_DEBUG_STREAM("pk(pre): " << endl << Pk << endl);
        ROS_DEBUG_STREAM("wk(pre): " << endl << wk << endl);

        //normalize weights
        float weight_tally = 0;
        float old_weight;


        //sum weights
        for(int i = 0; i < numTargets_Jk_k_minus_1; i ++)
        {
            index = (L) * numTargets_Jk_k_minus_1 + i;
            weight_tally = weight_tally + wk(index);
        }


        //divide sum weights
        for(int i = 0; i < numTargets_Jk_k_minus_1; i ++)
        {
            index = (L) * numTargets_Jk_k_minus_1 + i;
            old_weight = wk(index);
//            float measZx = Z_k(0,i);
//            float measZy = Z_k(1,i);
            wk(index) = old_weight / ( clutter_intensity(X_k(0,i),X_k(2,i))+ weight_tally);
//            wk(index) = old_weight / ( weight_tally);
        }

    }

    ROS_DEBUG_STREAM("wk(post): " << endl << setprecision(3) << wk << endl);
    ROS_DEBUG_STREAM("mk(post): " << endl << setprecision(3) << mk << endl);
    ROS_DEBUG_STREAM("Pk(post): " << endl << setprecision(3) << Pk << endl);

}

void PhdFilter::phd_prune()
{
    ROS_DEBUG("======= 4. prune ======= \n");

    //get location of maximum
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXf I_weights;
    Eigen::MatrixXd I, indexOrder;
    I = Eigen::MatrixXd(1,0);
    I_weights = Eigen::MatrixXf(1,0);
    indexOrder = Eigen::MatrixXd(1,NUM_DRONES);
    int index_counter = 0;
    int j = 0;
    int search_index = 0;
    int update_counter = 0;

    Eigen::MatrixXf P_bar_sum, P_val;
    P_val = Eigen::MatrixXf(n_state, n_state);
    P_bar_sum = Eigen::MatrixXf::Zero(n_state, n_state);


    //store for missed detection value
    for(int i = 0; i < NUM_DRONES; i++)
    {
        wk_bar(i) = wk_bar_fixed(i);
        mk_bar.block(0,i, n_state,1) = mk_bar_fixed.block(0,i, n_state,1);
        Pk_bar.block(0,n_state*i, n_state,n_state) = Pk_bar_fixed.block(0,n_state*i, n_state,n_state);
    }


    //find weights threshold
    for(int i = 0; i < wk.cols(); i ++)
    {
        if(wk(i) > 0.1)
        {
            index_counter++;
            I.conservativeResize(1,index_counter);
            I_weights.conservativeResize(1,index_counter);
            I(0,index_counter-1) = i;
            I_weights(0,index_counter-1) = wk(i);
        }
    }

    ROS_DEBUG_STREAM("index_weight is of size " << I.rows() << "x" << I.cols() << ",  I: "  << I << endl);


    for (int i = 0; i < NUM_DRONES; i++)
    {

        ROS_DEBUG("I length: %lu",I.cols());
        if(I.cols() == 0)
        {
            break;
        }

        update_counter++;
        //get max weight index

        float max = I_weights.maxCoeff(&maxRow, &maxCol);
        j = int(I(maxCol));
        //    ROS_DEBUG_STREAM("Max w: " << max <<  ", at I_index: " << maxCol << ", w_index: " << j << endl;

        //store index
        indexOrder(i) = j;

        //update w_k_bar,m_k_bar
//        wk_bar(i) = wk(j);
//        mk_bar.block<4,1>(0,i) = mk.block<4,1>(0,j);
//        Pk_bar.block<4,4>(0,4*i) = Pk.block<4,4>(0,4*j);

        int target_index = int(j%NUM_DRONES);
        wk_bar(target_index) = wk(j);
        mk_bar.block(0,target_index, n_state,1) = mk.block(0,j, n_state,1);
        Pk_bar.block(0,n_state*target_index, n_state,n_state) = Pk.block(0,n_state*j, n_state,n_state);


        //remove index that is same index multiple
        search_index = j%numTargets_Jk_k_minus_1;
        //    ROS_DEBUG_STREAM("search index:" << search_index <<  endl;
        for (int i =0; i < I.cols(); i++)
        {
            //        ROS_DEBUG_STREAM("I(i): " << int(I(i)) << " ...I(i)%3: " << int(I(i))%numTargets_Jk_k_minus_1 << endl;
            if( int(I(i))%numTargets_Jk_k_minus_1 == search_index )
            {

                //remove this index from I, Iweight
                        ROS_DEBUG_STREAM("removing index:" << i << endl);
                removeColumn(I,i);
                removeColumnf(I_weights,i);
                        ROS_DEBUG_STREAM("remaining I:" << I << endl);

                //to prevent skipping when [4 7 8 10 11] search index = 1... 4,7 causes to skip
                i--;

            }
        }

        ROS_DEBUG("I length: %lu",I.cols());
        if(I.cols() == 0)
        {
            break;
        }

        //    ROS_DEBUG_STREAM("I: "  << I << endl;
    }
    ROS_DEBUG_STREAM("indexOrder: "  << indexOrder << endl);

    //=============== rearrange index ===============
    if (indexOrder.cols() > 0)
    {

        Eigen::MatrixXd newIndex;
        newIndex = Eigen::MatrixXd(1,indexOrder.cols());

        //bring down index to numdrone range
        for(int i = 0; i <update_counter; i++  )
        {
            newIndex(i) = int(indexOrder(i))%numTargets_Jk_k_minus_1;
            // if(newIndex(i) == 0)
            // {
            //     newIndex(i) = 0; //unncessary bc index starts at 0
            // }
        }

        for(int i = 0; i <update_counter; i++  )
        {
            if(newIndex(i) >  NUM_DRONES)
            {
                newIndex(i) = int(newIndex(i))%NUM_DRONES;
            }
        }

        ROS_DEBUG_STREAM("newIndex: "  << newIndex << endl);

        int sortedIndex = 0;

//        ROS_DEBUG_STREAM("wk_bar: "  << wk_bar << endl;
//        ROS_DEBUG_STREAM("mk_bar: "  << endl << mk_bar << endl;
//        ROS_DEBUG_STREAM("wk_bar_fixed: "  << wk_bar_fixed << endl;
//        ROS_DEBUG_STREAM("mk_bar_fixed: "  << endl << mk_bar_fixed << endl;

        //sort highest weight to correct association
        for(int i = 0; i <NUM_DRONES; i++  )
        {
//            sortedIndex = newIndex(i);
//            wk_bar_fixed(sortedIndex) = wk_bar(i);
//            mk_bar_fixed.block<4,1>(0,sortedIndex) = mk_bar.block<4,1>(0,i);
//            Pk_bar_fixed.block<4,4>(0,4*sortedIndex) = Pk_bar.block<4,4>(0,4*i);

            wk_bar_fixed(i) = wk_bar(i);
            mk_bar_fixed.block(0,i, n_state,1) = mk_bar.block(0,i, n_state,1);
            Pk_bar_fixed.block(0,n_state*i, n_state,n_state) = Pk_bar.block(0,n_state*i, n_state,n_state);
        }

        ROS_DEBUG("updated");
    }



    numTargets_Jk_minus_1 = wk_bar_fixed.cols();
    //  ROS_DEBUG_STREAM("Pk_bar_fixed: " << endl << setprecision(3) << Pk_bar_fixed << endl;
    ROS_DEBUG("end prune");


}


void PhdFilter::phd_state_extract()
{

    ROS_DEBUG("============ 5. extract ============= ");
    Eigen::MatrixXf velocity, position;
    velocity = Eigen::MatrixXf(2,1);
    position = Eigen::MatrixXf(2,1);
    float gain_fine_tuned = 1;


    //update state for next iterations
    wk_minus_1 = wk_bar_fixed;
    mk_minus_1 = mk_bar_fixed;
    Pk_minus_1 = Pk_bar_fixed.cwiseAbs();

    
    ROS_DEBUG_STREAM("--- X_k: " << endl << X_k << endl);
    X_k = mk_minus_1;
    if (k_iteration > 3)
    {
        for (int i = 0; i < wk_bar_fixed.cols(); i++)
        {
            position.block<1, 1>(0, 0) = (X_k.block<1,1>(0,i) - X_k_previous.block<1,1>(0,i));
            position.block<1, 1>(1, 0) = (X_k.block<1,1>(2,i) - X_k_previous.block<1,1>(2,i));
            velocity = position/ (dt_cam*gain_fine_tuned) ;
            mk_minus_1.block<1,1>(1,i) = velocity.block<1,1>(0,0);
            mk_minus_1.block<1,1>(3,i) = velocity.block<1,1>(1,0);
            ROS_DEBUG_STREAM("--- position: " << endl << position << endl);
            ROS_DEBUG_STREAM("--- dt: " << endl << dt << endl);
            ROS_DEBUG_STREAM("--- velocity: " << endl << velocity << endl);
        }

    }
    X_k = mk_minus_1;
    X_k_previous = X_k;
}



float PhdFilter::clutter_intensity(const float ZmeasureX, const float ZmeasureY)
{
    float xMin = 0;
    float xMax = 224;
    float yMin = 0;
    float yMax = 224;
    float uniform_dist = 0;
    float clutter_intensity = 0;
    float lambda = 12.5*pow(10,-8);
    float volume = 4*pow(10,6);

    if(ZmeasureX < xMin) return 0;
    else if (ZmeasureX > xMax) return 0;
    else if (ZmeasureY < yMin) return 0;
    else if (ZmeasureY > yMax) return 0;
    else
    {
        uniform_dist = 1 / ( (xMax - xMin)*(yMax - yMin) ); // Convert this to a constant initialized at startup

    }

    clutter_intensity = lambda * volume * uniform_dist;

    return clutter_intensity;
}

void PhdFilter::removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void PhdFilter::removeColumnf(Eigen::MatrixXf& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

void PhdFilter::update_A_matrix(float input_dt)
{
    A << 1,input_dt,0,0,
            0,1,0,0,
            0,0,1,input_dt,
            0,0,0,1;
}

void PhdFilter::update_F_matrix(float input_dt)
{
    ROS_DEBUG_STREAM("This should not happen !\n");
    return;
}
