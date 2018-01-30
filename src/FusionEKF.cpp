#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;




/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  // cout << "Initialize FusionEKF" << endl;
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
  			  0, 1, 0, 0;




   //acceleration noise components
   noise_ax = 9;
   noise_ay = 9;

  //cout << "done" << endl;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: ";
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    // initialize matrices
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "RADAR" << endl;
      VectorXd measurement = measurement_pack.raw_measurements_;
      ekf_.x_ = tools.convert_polar_to_cartesian(measurement);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "LASER" << endl;
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      // assume null initial velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // set initial covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    ekf_.F_ = MatrixXd(4,4);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    // cout << "Init Completed" << endl;
    return;
  }

  cout << "Filter run..." << endl;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //const float noise_ax = 9;
   //const noise_ay = 9;

  const long long measurement_ts = measurement_pack.timestamp_;
  float dt = (measurement_ts - previous_timestamp_) / 1.e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "F..." << endl;

   ekf_.F_ << 1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;

   cout << "F" << endl;
   //2. Set the process covariance matrix Q
   const float dt2 = dt * dt;
   const float dt4 = dt2 * dt2 / 4;
   const float dt3 = dt2 * dt / 2;

   ekf_.Q_ = MatrixXd(4,4);
   ekf_.Q_ << dt4  * noise_ax, 0, dt3  * noise_ax, 0,
           0, dt4  * noise_ay, 0, dt3  * noise_ay,
           dt3 * noise_ax, 0, dt2 * noise_ax, 0,
           0, dt3 * noise_ay, 0, dt2 * noise_ay;

  cout << "Q" << endl;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/


  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
