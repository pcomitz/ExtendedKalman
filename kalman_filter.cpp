#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  // phc from L5_12
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //from L5_12 for LASER
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  //std::cout<<"Update: new estimate:"<<x_<<std::endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

   //z is measurement_pack.raw_measurements_
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  /* 
    ***KEY*** 
    In C++, atan2() returns values between -pi and pi. When calculating 
    phi in y = z - h(x) for radar measurements, the resulting angle 
    phi in the y vector should be adjusted so that it is between -pi and pi.
    The Kalman filter is expecting small angle values between the 
    range -pi and pi. HINT: when working in radians, you can add
    2π or subtract 2π until the angle is within the desired range.
  */

   VectorXd h(3); 
   h<<0,0,0;

  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  //calculate rho, phi
  // error - use sqrt
  double rho = sqrt(px*px + py*py);
  double phi = atan2(py,px);
  double rho_dot;

  //check for divide by 0
  if (rho < 0.0001) {
    std::cout << "Error while converting vector x_ to polar coordinates: Division by Zero" << std::endl;
  }
  else {
    rho_dot = (px*vx +py*vy)/rho;
  }

  // y = z -Hx'
  // use h = rho,phi,rho_dot
  h << rho, phi, rho_dot; 
  VectorXd y = z - h;

  //key step here
  // check for phi value in y vector between -pi and pi
  if(y(1) > M_PI)
  { 
    while(y(1) > M_PI)
    {
      std::cout << "y(1) > M_PI\n";
      y(1) -= 2*M_PI;
    } 
  }
  else if (y(1) < -M_PI)
  {
    while (y(1) < -M_PI)
    {
      std::cout << "y(1) < -M_PI\n";
      y(1) += 2 * M_PI;
    }
  }

  /* - this blows up
  //same as above , except R = R_radar_
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  
  // identical to LASER
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  std::cout<<"UpdateEKF: new estimate:"<<x_<<std::endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  */

  MatrixXd Ht = H_.transpose();
  
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}