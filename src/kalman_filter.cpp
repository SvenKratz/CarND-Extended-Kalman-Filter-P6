#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;



// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;              // state
  P_ = P_in;              // cov. matrix
  F_ = F_in;              // state trans. matrix
  H_ = H_in;              // measurement matrix
  R_ = R_in;              // measurement cov. matrix
  Q_ = Q_in;              // proccess cov. matrix
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // standard KF update
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  UpdateCommon(y);

  // // update equations
  // MatrixXd Ht = H_.transpose();
  // MatrixXd S = H_ * P_ * Ht + R_;
  // MatrixXd Si = S.inverse();
  // MatrixXd PHt = P_ * Ht;
  // MatrixXd K = PHt * Si;
  //
  // //new estimate
  // x_ = x_ + (K * y);
  // long x_size = x_.size();
  // MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // P_ = (I - K * H_) * P_;
}


// used for radar measurements
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // convert state from cartesian to polar in order to apply
  // taylor approximation
  VectorXd h = tools.convert_cartesian_to_polar(x_);
  VectorXd y = z - h;

  // normalize angle
  y[1] = atan2(sin(y(1)), cos(y(1)));

  UpdateCommon(y);

}

// update equations common to both types of Kalman filters
void KalmanFilter::UpdateCommon(const VectorXd &y)
{
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
