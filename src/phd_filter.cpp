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

  //predict existing
  phd_predict_existing();
  //construct
  phd_construct();
  //update
  phd_update();
  //prune
  phd_prune();
  //state extraction
  phd_state_extract();

  //draw track on image
  //draw_image();

  endTime = ros::Time::now();
//  ROS_WARN("total plan time: %f [sec]", (endTime - startTime).toSec());

}


void PhdFilter::initialize_matrix()
{
    ROS_INFO("first initialize matrix");
  //initialize
  Z_k = Eigen::MatrixXf::Zero(4,NUM_DRONES);
  ang_vel_k = Eigen::MatrixXf::Zero(3,1);

  mk_minus_1 = Eigen::MatrixXf::Zero(4,NUM_DRONES);
  wk_minus_1 = Eigen::MatrixXf::Zero(1,NUM_DRONES);
  Pk_minus_1 = Eigen::MatrixXf::Zero(4,NUM_DRONES*4);

  mk = Eigen::MatrixXf::Zero(4,NUM_DRONES+NUM_DRONES*NUM_DRONES);
  wk = Eigen::MatrixXf::Zero(1,NUM_DRONES+NUM_DRONES*NUM_DRONES);
  Pk = Eigen::MatrixXf::Zero(4,4*(NUM_DRONES+NUM_DRONES*NUM_DRONES) );

  mk_bar = Eigen::MatrixXf::Zero(4,NUM_DRONES);
  wk_bar = Eigen::MatrixXf::Zero(1,NUM_DRONES);
  Pk_bar = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);

  mk_bar_fixed = Eigen::MatrixXf::Zero(4,NUM_DRONES);
  wk_bar_fixed = Eigen::MatrixXf::Zero(1,NUM_DRONES);
  Pk_bar_fixed = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);



  mk_k_minus_1 = Eigen::MatrixXf::Zero(4,NUM_DRONES);
  wk_k_minus_1 = Eigen::MatrixXf::Zero(1,NUM_DRONES);
  Pk_k_minus_1 = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);
  P_k_k = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);
  S = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);

  F = Eigen::MatrixXf::Zero(4,4);
  Q = Eigen::MatrixXf::Zero(4,4);
  R = Eigen::MatrixXf::Zero(4,4);
  K = Eigen::MatrixXf::Zero(4,4*NUM_DRONES);

  X_k = Eigen::MatrixXf::Zero(4,NUM_DRONES);

  B = Eigen::MatrixXf::Zero(4,3*NUM_DRONES);

  mk_k_minus_1_beforePrediction = Eigen::MatrixXf::Zero(4,NUM_DRONES);
}

void PhdFilter::set_num_drones(int num_drones_in)
{
  NUM_DRONES = num_drones_in;
}

void PhdFilter::initialize()
{
  Eigen::MatrixXf P_k_init;
  P_k_init = Eigen::MatrixXf(4,4);
  P_k_init <<
              10,0,0,0,
      0,10,0,0,
      0,0,5,0,
      0,0,0,5;


  for(int i = 0; i < Z_k.cols(); i ++)
  {

    //store Z into mk (x,y)
    mk_minus_1(0,i) = Z_k(0,i);
    mk_minus_1(1,i) = Z_k(1,i);
    mk_minus_1(2,i) = 0;
    mk_minus_1(3,i) = 0;


    //store pre-determined weight into wk (from matlab)
    wk_minus_1(i) = .0016;

    //store pre-determined weight into Pk (from paper)
    Pk_minus_1.block<4,4>(0,i*4) = P_k_init;
  }

  F << 1,0,dt,0,
      0,1,0,dt,
      0,0,1,0,
      0,0,0,1;

  //Q = sigma_v^2 * [ [1/4*dt^4*I2, 1/2*dt^3*I2]; [1/2*dt^3* I2, dt^2*I2] ]; %Process noise covariance, given in Vo&Ma.

  Q << 6.25, 0, 12.5, 0,
      0, 6.25, 0, 12.5,
      12.5, 0, 25, 0,
      0, 12.5, 0, 25;

  R << 100,0,0,0,
      0,100,0,0,
      0,0,100,0,
      0,0,0,100;

  numTargets_Jk_minus_1 = NUM_DRONES;
}

void PhdFilter::phd_predict_existing()
{
  ROS_INFO("======= 1. predict ======= \n");


  mk_k_minus_1_beforePrediction = mk_minus_1;

  wk_minus_1 = prob_survival * wk_minus_1;

//  cout << "B: " << B << endl;
//  cout << "u: " << ang_vel_k << endl;

  Eigen::MatrixXf ang_vel_temp;
  ang_vel_temp = Eigen::MatrixXf::Zero(4,3);

  for (int i = 0; i < mk_minus_1.cols(); i++)
  {
      //cout << "i: " << i << endl;

      ang_vel_temp= B.block<4,3>(0,3*i) * ang_vel_k * 0.1; //scale down
//      cout << "Bu: " << endl << ang_vel_temp << endl;
      // Ax + Bu
      mk_minus_1.block<4,1>(0,i) = F * mk_minus_1.block<4,1>(0,i) + ang_vel_temp;
  }

  Eigen::MatrixXf P_temp;
  P_temp = Eigen::MatrixXf(4,4);

  for(int j = 0; j < mk_minus_1.cols(); j ++)
  {
    P_temp = Pk_minus_1.block<4,4>(0,4*j);
    P_temp = Q + F* P_temp * F.transpose();
    Pk_minus_1.block<4,4>(0,4*j) = P_temp;
  }

//     cout << "P: " << endl << Pk_minus_1 << endl;
//     cout << "mk-1: " << endl << setprecision(3) << mk_minus_1 << endl;
//     cout << "wk-1: " << endl << setprecision(3) << wk_minus_1 << endl;


  wk_k_minus_1 = wk_minus_1;
  mk_k_minus_1 = mk_minus_1;
  Pk_k_minus_1 = Pk_minus_1;
  numTargets_Jk_k_minus_1 = numTargets_Jk_minus_1;
  //  ROS_INFO("size track: %d", numTargets_Jk_k_minus_1);
}

void PhdFilter::phd_construct()
{
  ROS_INFO("======= 2. construct ======= \n");
  Eigen::MatrixXf PHt, S_j, SChol, SCholInv, identity, w1;
  PHt = Eigen::MatrixXf(4,4);
  S_j = Eigen::MatrixXf(4,4);
  SChol = Eigen::MatrixXf(4,4);
  SCholInv = Eigen::MatrixXf(4,4);
  identity = Eigen::MatrixXf::Identity(4,4);
  w1 = Eigen::MatrixXf(4,4);

  for(int j = 0; j < numTargets_Jk_k_minus_1; j ++)
  {

    PHt = Pk_k_minus_1.block<4,4>(0,4*j);
    S_j = R + PHt;
    SChol = S_j.llt().matrixU();
    SCholInv = identity*SChol.inverse();
    w1 = PHt * SCholInv;
    K.block<4,4>(0,4*j) = w1 * SCholInv.transpose();
    S.block<4,4>(0,4*j) = S_j;

    //    cout << "SChol: " << endl << SChol << endl;
    //    cout << "SCholInv: " << endl << SCholInv << endl;

    P_k_k.block<4,4>(0,4*j) = Pk_k_minus_1.block<4,4>(0,4*j) - w1*w1.transpose();

  }

//    cout << "P_k_k: " << endl << P_k_k << endl;
  //    cout << "K: " << endl << K << endl;


}

void PhdFilter::phd_update()
{
  ROS_INFO("======= 3. update ======= \n");

  //1. set up matrix size
  wk = Eigen::MatrixXf::Zero(1,numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1);
  mk = Eigen::MatrixXf::Zero(4,numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1);
  Pk = Eigen::MatrixXf::Zero(4, 4 * (numTargets_Jk_k_minus_1 * detected_size_k + numTargets_Jk_k_minus_1));


  Eigen::MatrixXf thisZ, thisVel, meanDelta_pdf, meanDelta, cov_pdf;
  Eigen::MatrixXf w_new, w_new_exponent;
  thisZ = Eigen::MatrixXf(4,1);
  thisVel = Eigen::MatrixXf(2,1);
  meanDelta_pdf = Eigen::MatrixXf(2,1);
  meanDelta = Eigen::MatrixXf(2,1);
  cov_pdf = Eigen::MatrixXf(4,4);
  w_new = Eigen::MatrixXf(1,1);
  w_new_exponent = Eigen::MatrixXf(1,1);

  int index = 0;
  L = 0;

//  cout << "numTargets_Jk_k_minus_1: "  << numTargets_Jk_k_minus_1 << endl;

  //2. update first columns of m_k which corresponds to no new detections
  for (int i = 0; i < numTargets_Jk_k_minus_1; i++ )
  {
    wk(i) = (1-prob_detection) * wk_k_minus_1(i);
    mk.block<4,1>(0,i)  = mk_k_minus_1.block<4,1>(0,i);
    Pk.block<4,4>(0,4*i) = Pk_k_minus_1.block<4,4>(0,4*i);

  }

//  cout << "# of Z: "  << detected_size_k << ", # of Jk: "<< numTargets_Jk_k_minus_1 <<  endl;



  //3. update all combinations for measurements and targets
  for (int z = 0; z < detected_size_k; z++)
  {
    L = L+1;

    for (int j = 0; j < numTargets_Jk_k_minus_1; j++)
    {

      thisZ.block<2,1>(0,0) = Z_k.block<2,1>(0,z);

      index = (L) * numTargets_Jk_k_minus_1 + j; //3~11

      //update weight (multivar prob distr)
      meanDelta_pdf = thisZ.block<2,1>(0,0) - mk_k_minus_1.block<2,1>(0,j);



      cov_pdf = S.block<2,2>(0,4*j);
      w_new_exponent = -0.5 * meanDelta_pdf.transpose() * cov_pdf.inverse() * meanDelta_pdf ;

      double w_val = wk_k_minus_1(j) * pow(2*PI, -1) * pow(cov_pdf.determinant(),-0.5) * exp(w_new_exponent(0,0));
      wk(index)= w_val;

      //update mean
      mk.block<4,1>(0,index) = mk_k_minus_1.block<4,1>(0,j);
      meanDelta = thisZ.block<2,1>(0,0) - mk_k_minus_1.block<2,1>(0,j);
      mk.block<2,1>(0,index) =  mk_k_minus_1.block<2,1>(0,j) + K.block<2,2>(0,4*j)*meanDelta;
      //update cov
      Pk.block<4,4>(0,4*index) = P_k_k.block<4,4>(0,4*j);

    }


    //cout << "mk: " << endl << mk << endl;

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
      float measZx = Z_k(0,i);
      float measZy = Z_k(1,i);
      wk(index) = old_weight / ( clutter_intensity(measZx,measZy)+ weight_tally);

    }


  }

  cout << "wk: " << endl << setprecision(3) << wk << endl;
  cout << "mk: " << endl << setprecision(3) << mk << endl;
  //  cout << "Pk: " << endl << setprecision(3) << Pk << endl;

}

void PhdFilter::phd_prune()
{
  ROS_INFO("======= 4. prune ======= \n");

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

  Eigen::MatrixXf P_bar_sum, P_val;
  P_val = Eigen::MatrixXf(4,4);
  P_bar_sum = Eigen::MatrixXf::Zero(4,4);

  //  cout << "wk: "  << wk << endl;
  //find weights threshold
  for(int i = 0; i < wk.cols(); i ++)
  {
    if(wk(i) > 0.00000005)
    {
      index_counter++;
      I.conservativeResize(1,index_counter);
      I_weights.conservativeResize(1,index_counter);
      I(0,index_counter-1) = i;
      I_weights(0,index_counter-1) = wk(i);
    }
  }

  std::cout << "index_weight is of size " << I.rows() << "x" << I.cols() << ",  I: "  << I << endl;


  for (int i = 0; i < NUM_DRONES; i++)
  {
    //get max weight index

    float max = I_weights.maxCoeff(&maxRow, &maxCol);
    j = int(I(maxCol));
//    cout << "Max w: " << max <<  ", at I_index: " << maxCol << ", w_index: " << j << endl;

    //store index
    indexOrder(i) = j;

    //update w_k_bar,m_k_bar
    wk_bar(i) = wk(j);
    mk_bar.block<4,1>(0,i) = mk.block<4,1>(0,j);
    Pk_bar.block<4,4>(0,4*i) = Pk.block<4,4>(0,4*j);


    //remove index that is same index multiple
    search_index = j%numTargets_Jk_k_minus_1;
//    cout << "search index:" << search_index <<  endl;
    for (int i =0; i < I.cols(); i++)
    {
//        cout << "I(i): " << int(I(i)) << " ...I(i)%3: " << int(I(i))%numTargets_Jk_k_minus_1 << endl;
      if( int(I(i))%numTargets_Jk_k_minus_1 == search_index )
      {

        //remove this index from I, Iweight
//        cout << "removing index:" << i << endl;
        removeColumn(I,i);
        removeColumnf(I_weights,i);
//        cout << "remaining I:" << I << endl;

        //to prevent skipping when [4 7 8 10 11] search index = 1... 4,7 causes to skip
        i--;

      }
    }

    //    cout << "I: "  << I << endl;
  }
  cout << "indexOrder: "  << indexOrder << endl;

  //=============== rearrange index ===============
  if (indexOrder.cols() > 0)
  {

    Eigen::MatrixXd newIndex;
    newIndex = Eigen::MatrixXd(1,indexOrder.cols());

    //bring down index to numdrone range
    for(int i = 0; i <indexOrder.cols(); i++  )
    {
      newIndex(i) = int(indexOrder(i))%numTargets_Jk_k_minus_1;
      if(newIndex(i) == 0)
      {
        newIndex(i) = 0; //unncessary bc index starts at 0
      }
    }

    for(int i = 0; i <indexOrder.cols(); i++  )
    {
      if(newIndex(i) >  NUM_DRONES)
      {
        newIndex(i) = int(newIndex(i))%NUM_DRONES;
      }
    }

    cout << "newIndex: "  << newIndex << endl;

    int sortedIndex = 0;
    //sort highest weight to correct association
    for(int i = 0; i <indexOrder.cols(); i++  )
    {
      sortedIndex = newIndex(i);
      wk_bar_fixed(sortedIndex) = wk_bar(i);
      mk_bar_fixed.block<4,1>(0,sortedIndex) = mk_bar.block<4,1>(0,i);
      Pk_bar_fixed.block<4,4>(0,4*sortedIndex) = Pk_bar.block<4,4>(0,4*i);
    }
  }



  /*
  //sum weight
  for(int i = 0; i < numTargets_Jk_k_minus_1; i++)
  {
    float weight_temp = 0;
    for(int k = 0; k < NUM_DRONES; k ++)
    {
      weight_temp = weight_temp + wk( (i+1)*numTargets_Jk_k_minus_1 + k);
    }
    wk_bar(i) = weight_temp;

//    wk_bar(i) = wk( (i+1)*numTargets_Jk_k_minus_1) + wk( (i+1)*numTargets_Jk_k_minus_1 + 1) + wk( (i+1)*numTargets_Jk_k_minus_1 + 2) ;
  }

  //weight * state
  int index = 0;
  Eigen::MatrixXf m_bar_sum;
  m_bar_sum = Eigen::MatrixXf(4,1);



  for(int i = 0; i < numTargets_Jk_k_minus_1; i++)
  {
    m_bar_sum = Eigen::MatrixXf::Zero(4,1);
    index = (i+1)*NUM_DRONES;
    for(int k =0; k < NUM_DRONES; k++ )
    {
      m_bar_sum = m_bar_sum + wk(index+k) * mk.block<4,1>(0,index+k);
    }

//    m_bar_sum = wk(index) * mk.block<4,1>(0,index) + wk(index+1) * mk.block<4,1>(0,index+1) + wk(index+2) * mk.block<4,1>(0,index+2)  ;
    mk_bar.block<4,1>(0,i) = 1/wk_bar(i) * m_bar_sum;
  }
  cout << "wk_bar: " << endl << wk_bar << endl;
  cout << "mk_bar: " << endl << mk_bar << endl;
*/

  /*
  Eigen::MatrixXf P_bar_sum, P_val, delta_m;
  P_val = Eigen::MatrixXf(4,4);
  delta_m = Eigen::MatrixXf(4,1);
  P_bar_sum = Eigen::MatrixXf::Zero(4,4);

  int indexCount = NUM_DRONES-1;

  //update cov bar
  for(int i = 0; i < numTargets_Jk_k_minus_1; i ++)
  {

    //clear psum
    P_bar_sum = Eigen::MatrixXf::Zero(4,4);

    for(int j = 0; j < numTargets_Jk_k_minus_1; j ++)
    {
      indexCount = indexCount + 1;
      //get delta_m
      delta_m.block<4,1>(0,0) =  mk_bar.block<4,1>(0,i) - mk.block<4,1>(0,indexCount);
      //get pval
      P_val.block<4,4>(0,0) = Pk.block<4,4>(0,4*indexCount) + delta_m * delta_m.transpose();
      P_bar_sum.block<4,4>(0,0) = P_bar_sum + (wk(indexCount) * P_val);

    }

    //pval  / weight
    Pk_bar.block<4,4>(0,4*i) = P_bar_sum / wk_bar(i);

  }
  */

  //cout << "Pk_bar: " << endl << Pk_bar << endl;


  numTargets_Jk_minus_1 = wk_bar_fixed.cols();
  //  cout << "Pk_bar_fixed: " << endl << setprecision(3) << Pk_bar_fixed << endl;



}


void PhdFilter::phd_state_extract()
{

  Eigen::MatrixXf velocity;
  velocity = Eigen::MatrixXf(2,1);

  //update state for next iterations
  wk_minus_1 = wk_bar_fixed;
  mk_minus_1 = mk_bar_fixed;
  Pk_minus_1 = Pk_bar_fixed.cwiseAbs();

  if (k_iteration > 3)
  {
      for (int i = 0; i < wk_bar_fixed.cols(); i++)
      {
          velocity = (mk_minus_1.block<2,1>(0,i) - mk_k_minus_1_beforePrediction.block<2,1>(0,i))/ dt ;
          mk_minus_1.block<2,1>(2,i) = velocity;
      }

  }


  X_k = mk_minus_1;
  cout << "--- X_k: " << endl << X_k << endl;


}



float PhdFilter::clutter_intensity(const float ZmeasureX, const float ZmeasureY)
{
  float xMin = 0;
  float xMax = 360;
  float yMin = 0;
  float yMax = 240;
  float uniform_dist = 0;
  float clutter_intensity = 0;
  float lambda = 12.5*pow(10,-6);
  float volume = 4*pow(10,6);

  if(ZmeasureX < xMin) return 0;
  else if (ZmeasureX > xMax) return 0;
  else if (ZmeasureY < yMin) return 0;
  else if (ZmeasureY > yMax) return 0;
  else
  {
    uniform_dist = 1 / ( (xMax - xMin)*(yMax - yMin) );

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
