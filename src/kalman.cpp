#include <multi_robot_tracking/kalman.h>

using namespace std;

Kalman::Kalman(const float& px, const float& py, const float& vx, const float& vy, TrackerParam params)
{
  /*
  * params.R << 2, 0, 0, 2;
  * params.T << 100, 100;
  * 
  * params.P_0 << 10, 0, 0, 0,
  *               0, 20, 0, 0,
  *               0, 0, 10, 0,
  *               0, 0, 0, 20;
  * 
  * params.principal_point << 180, 120;
  * params.focal_length = 120;
  */

  //CONTROL INPUT model Matrix
  B = Eigen::MatrixXf(4, 3);
  B << 0, 0, 0,
       0, 0, 0,
       0, 0, 0,
       0, 0, 0;

  //STATE OBSERVATION MATRIX
  C = Eigen::MatrixXf(2, 4);
  C << 1, 0, 0, 0,
       0, 0, 1, 0;

  //INITIAL COVARIANCE MATRIX
  //GAIN
  K = Eigen::MatrixXf(4, 2);

  //measurement noise covariance matrix
  R = params.R;

  T = params.T;

  f = params.focal_length;
  alpha = params.alpha_cam;
  c = params.principal_point;

  x << px, vx, py, vy;
  z = C * x;
  P = params.P_0;

  //cout << "P" << endl << P << endl;

}


void Kalman::predict(const float dt, const Eigen::Vector3f omega)
{
  Eigen::Matrix4f A;
  A << 1, dt, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, dt,
       0, 0, 0, 1;

  //Lets try an other way:
  Eigen::Matrix4f Q;
  Q << std::pow(dt, 2)*T(0)/2, 0, 0, 0,
       0, dt*T(1), 0, 0,
       0, 0, std::pow(dt, 2)*T(0)/2, 0,
       0, 0, 0, dt*T(1);


//  cout << "Q: " << endl << Q << endl;//Q has very low position variances and very high speed variances


  Eigen::Vector3f u = omega*dt;


  B(0,0) = ((z(0)-c(0))*(z(1)-c(1)))/f;
  B(0,1) = -(f*alpha + (z(0)-c(0))*(z(0)-c(0))/(f*alpha));
  B(0,2) = alpha*(z(1)-c(1));

  B(2,0) = (f + (z(1)-c(1))*(z(1)-c(1))/f);
  B(2,1) = -((z(0)-c(0))*(z(1)-c(1)))/(alpha*f);
  B(2,2) = -(z(0)-c(0))/alpha;

  B(1,0) = 0;
  B(1,1) = 0;
  B(1,2) = 0;

  B(3,0) = 0;
  B(3,1) = 0;
  B(3,2) = 0;
  
  if((isnan(B.array())).any()) // may happen if one track flies to infinity. The return is just to avoid a crash
  {
    ROS_FATAL("B contains NaNs");
    return;
  }
  if((isinf(B.array())).any())
  {
    ROS_FATAL("B contains infs");
    return;
  }

  Eigen::MatrixXf ang_vel;
  ang_vel = B*u;
  cout << "B: " << B << endl;
  cout << "u: " << u << endl;
  cout << "Bu: " << ang_vel << endl;

  x = A*x + ang_vel;

//  cout << "x predict:" << endl << x << endl;
  Eigen::MatrixXf F = Eigen::MatrixXf(4, 4);
  F(0,0) = (dt*u(1)*(2*c(0) - 2*z(0)))/f - (dt*u(0)*(c(1) - z(1)))/f + 1;       F(0,1) = dt;   F(0,2) = dt*u(2) - (dt*u(0)*(c(0) - z(0)))/f;                            F(0,3) = 0;
  F(2,0) = 0;                                                                   F(2,1) = 1;    F(2,2) = 0;                                                              F(2,3) = 0;
  F(1,0) = (dt*u(1)*(c(1) - z(1)))/f - dt*u(2);                                 F(1,1) = 0;    F(1,2) = (dt*u(1)*(c(0) - z(0)))/f - (dt*u(0)*(2*c(1) - 2*z(1)))/f + 1;  F(1,3) = dt;
  F(3,0) = 0;                                                                   F(3,1) = 0;    F(3,2) = 0;                                                              F(3,3) = 1;


  P = A * P * A.transpose() + Q;  //ttt

//  cout << "P: " << endl << P << endl;

  //the following bugs should not happen anymore, but I leave the checks in case some bug percists
  if(P.determinant() < 0)
  {
    ROS_FATAL("Predicted covariance determinant is negative! %f", P.determinant());
    exit(0);
  }
  if((isnan(P.array())).any())
  {
    ROS_FATAL("P contains NaNs");
    exit(0);
  }
  if((isinf(P.array())).any())
  {
    ROS_FATAL("P contains infs");
    exit(0);
  }

  //Error Measurement Covariance Matrix
  S = C * P * C.transpose() + R;
  //cout << "S: " << endl << S << endl;


  z = C * x;

  return;
}



void Kalman::update(const std::vector<Detection> detections, const std::vector<double> beta, double beta_0)
{

  K = P * C.transpose() * S.inverse();
  cout << "K:" << endl << K << endl;

  std::vector<Eigen::Vector2f> nus;
  for(uint i=0; i<detections.size(); i++)
  {
    Eigen::Vector2f nu_i = detections[i].getVect() - z;
    nus.push_back(nu_i);
  }

  Eigen::Vector2f nu;
  nu << 0, 0;
  for(uint i=0; i<detections.size(); i++)
  {
    nu += beta[i] * nus[i];
  }

  x = x + K * nu;

//  cout << "x update:" << endl << x << endl;

  Eigen::Matrix4f P_c;
  P_c = P - K * S * K.transpose(); //Changed here, there is an error in the PhD thesis! It should be - instead of +

  Eigen::Matrix4f P_tild;
  Eigen::Matrix2f temp_sum;
  temp_sum << 0, 0, 0, 0;

  for(uint i=0; i<detections.size(); i++)
  {
      temp_sum += beta[i]*nus[i]*nus[i].transpose();
  }
  temp_sum -= nu*nu.transpose();

  P_tild = K * temp_sum * K.transpose();

  P = beta_0*P + (1-beta_0)*P_c + P_tild;
  //cout << "P" << endl << P << endl;


  //the following bugs should not happen anymore, but I leave the checks in case some bug percists
  if(P.determinant() < 0)
  {
    ROS_FATAL("Update covariance determinant is negative! %f", P.determinant());
    exit(0);
  }
  if((isnan(P.array())).any())
  {
    ROS_FATAL("P contains NaNs");
    exit(0);
  }
  if((isinf(P.array())).any())
  {
    ROS_FATAL("P contains infs");
    exit(0);
  }


  z = C * x; //ttt

}


