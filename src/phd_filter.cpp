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
    ROS_INFO("iter: %d",k_iteration);

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
            5,0,0,0,
            0,2,0,0,
            0,0,5,0,
            0,0,0,2;


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
    Q = Q*0.45;  //0.01 works well for exp3

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
    ROS_INFO("======= 0. asynch predict ======= \n");
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
    

    //ROS_INFO("size mk-1: %lu, size B: %lu",mk_minus_1.cols(), B.cols());
    //ROS_INFO_STREAM("A:\n" << A << endl);
    for (int i = 0; i < mk_minus_1.cols(); i++)
    {
        float qAcc = sqrt((mk_k_minus_1(1, i))*(mk_k_minus_1(1, i)) + (mk_k_minus_1(3, i)*(mk_k_minus_1(3, i))));
        qAcc = ((qAcc*2)+1);
        // ROS_INFO_STREAM("!!!!!!!!!!QACC = " << qAcc);
        Eigen::MatrixXf Q_temp = Q;
        Q_temp = Q;// * qAcc;
        // ROS_INFO("iteration: %d", i);
        // ROS_INFO_STREAM("B:\n" << B.block(0,n_input*i, n_state,n_input)); 
        Bu_temp = B.block(0,n_input*i, n_state,n_input) * ang_vel_k; //
        // ROS_INFO_STREAM("Bu:\n" << Bu_temp); 
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

    // ROS_INFO_STREAM("WK|K-1:\n" << wk_k_minus_1 << endl);
    // ROS_INFO_STREAM("mK|K-1:\n" << mk_k_minus_1 << endl);
    // ROS_INFO_STREAM("PK|K-1:\n" << Pk_k_minus_1 << endl);
}


void PhdFilter::phd_construct()
{
    ROS_INFO("======= 2. construct ======= \n");
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
    ROS_INFO_STREAM("P_k_k: " << endl << P_k_k << endl);
    ROS_INFO_STREAM("K: " << endl << K << endl);
}

void PhdFilter::phd_update()
{
    ROS_INFO("======= 3. update ======= \n");

    //1. set up matrix size
    mahalDistance = Eigen::MatrixXf::Zero(1,numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1);
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
        mahalDistance(i) = 0;
    }

    for (int z = 0; z < detected_size_k; z++)
    {
        L = L+1;

        for (int j = 0; j < numTargets_Jk_k_minus_1; j++)
        {
            thisZ = Z_k.block(0,z, n_meas,1);

            index = (L) * numTargets_Jk_k_minus_1 + j; //3~11
            ROS_INFO("Index: %d, L: %d\n", index, L);
            //update weight (multivar prob distr)
            meanDelta_pdf = thisZ - H*mk_k_minus_1.block(0,j, n_state,1);
            ROS_INFO_STREAM("Mean Delta PDF: " << meanDelta_pdf << endl);

            cov_pdf = S.block(0,n_meas*j, n_meas,n_meas);
            ROS_INFO_STREAM("Cov PDF: " << cov_pdf << endl);
            w_new_exponent = -0.5 * (meanDelta_pdf.transpose() * cov_pdf.inverse() * meanDelta_pdf).transpose() * (meanDelta_pdf.transpose() * cov_pdf.inverse() * meanDelta_pdf) ;
            ROS_INFO_STREAM("W_new_exponent: " << w_new_exponent << endl);

            mahalDistance(index) = fabs(w_new_exponent(0));

            double q_val = wk_k_minus_1(j) * pow(2*PI, -1/2) * pow(cov_pdf.determinant(),-0.5) * exp(w_new_exponent(0,0));
            ROS_INFO_STREAM("q_val: " << q_val << endl);
            wk(index)= q_val;

            double w_numerator = prob_detection * wk_k_minus_1(j) * q_val;

            //update mean
            mk.block(0,index, n_state,1) = mk_k_minus_1.block(0,j, n_state,1);
            mk.block(0,index, n_state, 1) =  mk_k_minus_1.block(0,j, n_state,1) + K.block(0,n_meas*j, n_state,n_meas)*meanDelta_pdf;
            //update cov
            Pk.block(0,n_state*index, n_state,n_state) = P_k_k.block(0,n_state*j, n_state,n_state);

//            ROS_INFO("wk: %f",wk(index));
//            ROS_INFO_STREAM << "mk: "<< mk <<  endl;

        }

        ROS_INFO_STREAM("mk(pre): " << endl << mk << endl);
        ROS_INFO_STREAM("pk(pre): " << endl << Pk << endl);
        ROS_INFO_STREAM("wk(pre): " << endl << wk << endl);

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
            wk(index) = old_weight / (clutter_intensity(X_k(0,i),X_k(2,i))/20.0+ weight_tally);
//            wk(index) = old_weight / ( weight_tally);
        }

    }
    ROS_INFO_STREAM("Mahalanobis Distance: " << mahalDistance);
    ROS_INFO_STREAM("wk(post): " << endl << setprecision(3) << wk << endl);
    ROS_INFO_STREAM("mk(post): " << endl << setprecision(3) << mk << endl);
    ROS_INFO_STREAM("Pk(post): " << endl << setprecision(3) << Pk << endl);

}

void PhdFilter::phd_prune()
{

    ROS_INFO_STREAM("======= 4.Pruning =======");
    Eigen::MatrixXi::Index maxRow, maxCol;
    Eigen::MatrixXi I, I_copy;
    Eigen::MatrixXf I_weights;

    I = Eigen::MatrixXi(1, 0);
    I_copy = Eigen::MatrixXi(1, 0);
    I_weights = Eigen::MatrixXf(1, 0);

    int I_counter = 0;
    float weight_threshold = 0.1;
    float mahalanobis_threshold = 4.0;
    int l = 0;

    for(int i=0; i<wk.cols(); i++)
    {
        if(wk(i) > weight_threshold)
        {
            I_counter += 1;
            I.conservativeResize(1, I_counter);
            I_weights.conservativeResize(1, I_counter);
            I(0, I_counter-1) = i;
            I_weights(0, I_counter-1) = wk(i);
        }
    }
    I_copy = I;

    ROS_INFO_STREAM("I is:\n" << I << "\nWK is:\n" << wk);
    
    Eigen::MatrixXf wk_bar_fixed_k = Eigen::MatrixXf::Zero(1, 0);
    Eigen::MatrixXf mk_bar_fixed_k = Eigen::MatrixXf::Zero(4, 0);
    Eigen::MatrixXf Pk_bar_fixed_k = Eigen::MatrixXf::Zero(4, 0);
    Eigen::MatrixXi index_order = Eigen::MatrixXi::Zero(1, 0);

    Eigen::MatrixXf highWeights = Eigen::MatrixXf::Zero(1, I.cols());

    Eigen::MatrixXi L = Eigen::MatrixXi::Zero(1, I.cols());


    while(I.cols() != 0)
    {
        l++;
        ROS_INFO_STREAM("Current l: " << l);
        if(I_copy.cols() == 0)
            break;

        if(l > NUM_DRONES * n_meas)
            break;
        
        Eigen::MatrixXf highWeights = Eigen::MatrixXf::Zero(1, I.cols());
        for(int i=0; i<I_copy.cols(); i++)
        {
            highWeights(i) = wk(I_copy(i));
        }
        ROS_INFO_STREAM("HighWeights is: " << highWeights);

        float maxW = 0.0;
        int maxW_index = -1;
        for(int i=0; i<highWeights.cols(); i++)
        {
            if(maxW < highWeights(i))
            {
                maxW = highWeights(i);
                maxW_index = i;
            }
        }
        ROS_INFO_STREAM("maxW is: " << maxW << " and its index is " << maxW_index);

        int j = I_copy(maxW_index);
        L = Eigen::MatrixXi::Zero(1, 0);
        for(int i=0; i<I.cols(); i++)
        {
            int thisI = I(i);
            Eigen::MatrixXf deltaM = mk.block(0,thisI,n_state,1) - mk.block(0,j,n_state,1);
            float mahalanobis_distance = (deltaM.transpose() * Pk.block(0,n_state*thisI,n_state,n_state).inverse() * deltaM)(0);
            ROS_INFO_STREAM("index for mahalanobis distance: " << i << " in wk " << thisI << " distance: " << mahalanobis_distance);

            if(mahalanobis_distance < mahalanobis_threshold)
            {
                L.conservativeResize(1, L.cols()+1);
                L(L.cols()-1) = thisI;
                if(thisI-j == 0)
                {
                    //Need to do something here, seems unimportant
                }
            }
        }
        ROS_INFO_STREAM("L is: " << L);

        
        float w_bar_k_l = 0; // Sum of all associated weights
        for(int i=0; i<L.cols(); i++)
        {
            int thisI = L(i);
            w_bar_k_l += wk(thisI);
        }
        ROS_INFO_STREAM("w_bar_k_l is: " << w_bar_k_l);

        Eigen::MatrixXf m_bar_k_l = Eigen::MatrixXf::Zero(n_state,1);
        for(int i=0; i<L.cols(); i++)
        {
            int thisI = L(i);
            m_bar_k_l += (wk(thisI) * mk.block(0, thisI, n_state, 1)) / w_bar_k_l;
        }
        ROS_INFO_STREAM("m_bar_k_l is: " << m_bar_k_l);

        Eigen::MatrixXf pVal = Eigen::MatrixXf::Zero(n_state, n_state);
        for(int i=0; i<L.cols(); i++)
        {
            int thisI = L(i);
            Eigen::MatrixXf deltaM = m_bar_k_l.block(0, 0, n_state, 1) - mk.block(0,j,n_state,1);
            pVal += wk(thisI) * (Pk.block(0, n_state*thisI, n_state, n_state));
        }
        pVal = pVal/w_bar_k_l;
        ROS_INFO_STREAM("pVal is: " << pVal);

        Eigen::MatrixXf oldP = Pk.block(0, n_state*j, n_state, n_state);
        index_order.conservativeResize(1, index_order.cols()+1);
        index_order(index_order.cols()-1) = j;
        ROS_INFO_STREAM("index order: " << index_order << endl);
        ROS_INFO_STREAM("L: " << L << endl);
        // Remove all indices in L from I
        for(int i=0; i<L.cols(); i++)
        {
            ROS_INFO_STREAM("i in L.cols: " << i << endl);
            int thisI = L(i);
            for(int ii=0; ii<I.cols(); ii++)
            {
                ROS_INFO_STREAM("ii in I.cols: " << ii << endl);
                if(thisI == I(ii))
                {   
                    ROS_INFO_STREAM("Removing ThisI: " << thisI << " at index: " << ii);
                    removeColumni(I, ii);
                    removeColumni(I_copy, ii);
                }
            }
        }


        int deleteDroneIndex = j%NUM_DRONES;
        for(int i=0; i<I_copy.cols(); i++)
        {
            if(I_copy(i) % numTargets_Jk_k_minus_1 == deleteDroneIndex)
            {
                ROS_INFO_STREAM("Removing indices for drone: " << deleteDroneIndex << " at index: " << i);
                removeColumni(I_copy, i);
                removeColumni(I, i);
            } 
        }
        ROS_INFO_STREAM("New I: " << I);
        ROS_INFO_STREAM("New I_copy: " << I_copy);

        wk_bar_fixed_k.conservativeResize(1, wk_bar_fixed_k.cols()+1);
        wk_bar_fixed_k(wk_bar_fixed_k.cols()-1) = w_bar_k_l;
        ROS_INFO_STREAM("wk_bar_fixed_k:\n" << wk_bar_fixed_k);

        mk_bar_fixed_k.conservativeResize(n_state, mk_bar_fixed_k.cols()+1);
        mk_bar_fixed_k.block(0, mk_bar_fixed_k.cols()-1, n_state, 1) = m_bar_k_l;
        ROS_INFO_STREAM("mk_bar_fixed_k:\n" << mk_bar_fixed_k);

        Pk_bar_fixed_k.conservativeResize(n_state, Pk_bar_fixed_k.cols()+n_state);
        Pk_bar_fixed_k.block(0, Pk_bar_fixed_k.cols()-n_state,n_state,n_state) = pVal;
        ROS_INFO_STREAM("Pk_bar_fixed_k:\n" << Pk_bar_fixed_k);
    }

    ROS_INFO_STREAM("wk_bar_fixed_k is:\n" << wk_bar_fixed_k << "\n");
    ROS_INFO_STREAM("mk_bar_fixed_k is:\n" << mk_bar_fixed_k << "\n");
    ROS_INFO_STREAM("wk_bar_fixed_k is:\n" << wk_bar_fixed_k << "\n");
    // Sorting out mixed association
    Eigen::MatrixXi newIndex = index_order;
    for(int i=0; i<newIndex.cols(); i++)
    {
        newIndex(i) = newIndex(i)%numTargets_Jk_k_minus_1;
    }

    for(int i=0; i<newIndex.cols(); i++)
    {
        if(newIndex(i) == 0)
        {
            //Do nothing here, Matlab uses indices as 1, 2, 3 and cpp uses 0, 1, 2 !
            newIndex(i) == newIndex(i);
        }
    }

    for(int i=0; i<newIndex.cols(); i++)
    {
        if(newIndex(i) >= NUM_DRONES)
        {
            newIndex(i) = newIndex(i) % NUM_DRONES;
        }
    }

    for(int i=0; i<newIndex.cols(); i++)
    {
        if(i > NUM_DRONES)
        {
            continue;
        }

        int sortedIndex = newIndex(i);
        wk_bar_fixed.block(0, sortedIndex, 1, 1) = wk_bar_fixed_k.block(0, i, 1, 1);
        mk_bar_fixed.block(0, sortedIndex, n_state, 1) = mk_bar_fixed_k.block(0, i, n_state, 1);
        Pk_bar_fixed.block(0, n_state*sortedIndex, n_state, n_state) = Pk_bar_fixed_k.block(0, n_state*i, n_state, n_state);
    }
    ROS_INFO_STREAM("wk_bar_fixed is:\n" << wk_bar_fixed << "\n");
    ROS_INFO_STREAM("mk_bar_fixed is:\n" << mk_bar_fixed << "\n");
    ROS_INFO_STREAM("Pk_bar_fixed is:\n" << Pk_bar_fixed << "\n");

    numTargets_Jk_minus_1 = wk_bar_fixed.cols();
    ROS_INFO_STREAM("numTargets_Jk_minus_1: " << numTargets_Jk_minus_1);

}

void PhdFilter::phd_state_extract()
{

    ROS_INFO("============ 5. extract ============= ");
    Eigen::MatrixXf velocity, position;
    velocity = Eigen::MatrixXf(2,1);
    position = Eigen::MatrixXf(2,1);
    float gain_fine_tuned = 1;
    float weight_threshold_for_extraction = 0.5;
    //update state for next iterations
    // wk_minus_1 = wk_bar_fixed;
    // mk_minus_1 = mk_bar_fixed;
    // Pk_minus_1 = Pk_bar_fixed.cwiseAbs();

    
    // ROS_DEBUG_STREAM("--- X_k: " << endl << X_k << endl);
    // X_k = mk_minus_1;
    if(k_iteration > 3)
    {
        // wk_minus_1 = wk_bar_fixed;
        // mk_minus_1 = mk_bar_fixed;
        // Pk_minus_1 = Pk_bar_fixed.cwiseAbs();
        for(int i=0; i<wk_bar_fixed.cols(); i++)
        {
            if(wk_bar_fixed(i) < weight_threshold_for_extraction)
            {
                X_k.block(0, i, n_state, 1) = mk_minus_1.block(0, i, n_state, 1);
                mk_bar_fixed.block(0, i, n_state, 1) = mk_minus_1.block(0, i, n_state, 1);
                wk_bar_fixed(i) = wk_minus_1(i);
                Pk_bar_fixed.block(0, n_state*i, n_state, n_state) = Pk_minus_1.block(0, n_state*i, n_state, n_state).cwiseAbs();
            }
            else
            {
                X_k.block(0, i, n_state, i) = mk_bar_fixed.block(0, i, n_state, i);
            }
        }
        ROS_INFO_STREAM("mK in extract: \n" << mk_minus_1);
        ROS_INFO_STREAM("wK in extract: \n" << wk_bar_fixed);
        ROS_INFO_STREAM("PK in extract: \n" << Pk_bar_fixed);
        ROS_INFO_STREAM("XK in extract: \n" << X_k);
    }
    else
    {
        wk_minus_1 = wk_bar_fixed;
        mk_minus_1 = mk_bar_fixed;
        Pk_minus_1 = Pk_bar_fixed.cwiseAbs();
    }
    if (k_iteration > 3)
    {
        for (int i = 0; i < wk_bar_fixed.cols(); i++)
        {
            position.block<1, 1>(0, 0) = (X_k.block<1,1>(0,i) - X_k_previous.block<1,1>(0,i));
            position.block<1, 1>(1, 0) = (X_k.block<1,1>(2,i) - X_k_previous.block<1,1>(2,i));
            velocity = position/ (dt_cam*gain_fine_tuned) ;
            mk_minus_1.block<1,1>(1,i) = velocity.block<1,1>(0,0);
            mk_minus_1.block<1,1>(3,i) = velocity.block<1,1>(1,0);
            ROS_INFO_STREAM("--- position: " << endl << position << endl);
            ROS_INFO_STREAM("--- dt: " << endl << dt << endl);
            ROS_INFO_STREAM("--- velocity: " << endl << velocity << endl);
        }

    }
    wk_minus_1 = wk_bar_fixed;
    mk_minus_1 = mk_bar_fixed;
    Pk_minus_1 = Pk_bar_fixed.cwiseAbs();
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

void PhdFilter::removeColumni(Eigen::MatrixXi& matrix, unsigned int colToRemove)
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
    ROS_INFO_STREAM("This should not happen !\n");
    return;
}
