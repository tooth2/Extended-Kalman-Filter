#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   # prediction
   x= (F*x)+u
    // KF Prediction step
   P = F * P * Ft + Q;
   */
    x_ = F_ * x_; //sensor noise u=0
    P_ = F_ * P_ * F_.transpose() + Q_; // predict new covariance


}

void KalmanFilter::Update(const VectorXd &z) {
    
  /**
   * TODO: update the state by using Kalman Filter equations
   # measurement update
   y = Z-(H*x)
   S = H*P*Ht + R
   K = P*Ht*Si
   # new state
   x = x+ (K*y)
   P= (I- (K*H)) *P
   */
    // KF Measurement update step
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    
    MatrixXd PHt = P_ * H_.transpose();
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();
    
    //update the state new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    float px, py, vx, vy;
    px = x_[0];
    py = x_[1];
    vx = x_[2];
    vy = x_[3];
    
    float rho, phi, rho_dot;
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);
    
    // if rho is very small, set it to 0.0001 to avoid division by 0
    if(rho < 0.00001){
        //cout << "Small prediction value - reassigning Rho_p to 0.00001 to avoid division by zero";
        rho = 0.00001;
    }
    // convert prediction to a angular velocity
    rho_dot = (px * vx + py * vy) / rho;
    
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, phi, rho_dot;
    
    VectorXd y = z - z_pred;
    // const float PI = 3.14159265;
    // normalize the angle between -pi to pi
    //Adjust the value of angle if it is outside of [-PI, PI]
    if(y(1) > M_PI){
        y(1) -= 2*M_PI;
    }
    
    else if (y(1) < -M_PI){
        y(1) += 2*M_PI;
    }
    
    //same as KalmanFilter::Update()
    MatrixXd PHt = P_ * H_.transpose();
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();
    
    //update the state new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
