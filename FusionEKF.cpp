#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4); //jacobian

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    //create a 4D state vector, just initialize no values of the x state
    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4,4);  // state covariance matrix P
    //Adjust the variance values to reflect uncertainty in initial state
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    ekf_.F_ = MatrixXd(4,4); // the initial transition matrix F
    ekf_.H_ = MatrixXd(2,4); // measurement matrix

    //Initalize the state transition matrix F
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
  
    cout << "EKF: Extended Kalman Filter Initialization " << endl;
      
      // first measurement
      //cout << "EKF: " << endl;
      //ekf_.x_ = VectorXd(4);
      //ekf_.x_ << 1, 1, 1, 1;
      
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float rho = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        float rho_dot = measurement_pack.raw_measurements_[2];//not used
        //phi is not the direction of the speed, so set vx and vy to 0 to meet same condition as Lidar case
        ekf_.x_ << rho*cos(phi), rho*sin(phi),
                   0,
                   0;
    }
      
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
        
        //float px = measurement_pack.raw_measurements_[0];
        //float py = measurement_pack.raw_measurements_[1];
        //ekf_.x_(0) = px;
        //ekf_.x_(1) = py;
        // set the state with the initial location and zero velocity
        ekf_.x_ << measurement_pack.raw_measurements_[0],
                   measurement_pack.raw_measurements_[1],
                   0,
                   0;
        //std::cout << "Laser Initial state: " << ekf_.x_ ;
    }
    //first meansure: save time stamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
    
    // compute the time elapsed between current and previous measurements
    // dt - expressed in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    //save the current timestamp for the next predict cycle
    previous_timestamp_ = measurement_pack.timestamp_;
    
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt3*dt;
    
    //update the state transition matrix F so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    //set the process noise covariance matrix Q
    ekf_.Q_ = MatrixXd(4,4);
    //set the acceleration noise components for Q
    short noise_ax = 9;
    short noise_ay = 9;
    //update the process covariance matrix Q
    ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
               0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
               dt3/2*noise_ax, 0, dt2*noise_ax, 0,
               0, dt3/2*noise_ay, 0, dt2*noise_ay;
    
    //predict : call the EKF predict() function
    ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates 1) Turn off Radar
    //Calculate the Jacobian and update the EKF state matrix H
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      //Initialize the EKF(Radar) covariance matrix R
      //ekf_.R_ = MatrixXd(3,3);
      //update the covariance matrix R
      ekf_.R_ = R_radar_;
      // measurement update
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates --> 2)Turn off Laser (Lidar)
      //Set the EKF object to use the LASER sensor matrix H
      ekf_.H_ = H_laser_;
      //Initialize the EKF(Laser) covariance matrix R
      //ekf_.R_ = MatrixXd(2,2);
      //update the covariance matrix R
      ekf_.R_ = R_laser_;
      // measurement update
      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
