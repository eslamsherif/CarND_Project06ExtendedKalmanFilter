#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init() {
  
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);
  F_ = MatrixXd(4, 4);
  H_ = MatrixXd(2, 4);
  R_ = MatrixXd(2, 2);
  Q_ = MatrixXd(4, 4);
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  
  x_ = VectorXd(4);
  P_ = MatrixXd(4, 4);
  F_ = MatrixXd(4, 4);
  H_ = MatrixXd(2, 4);
  R_ = MatrixXd(2, 2);
  Q_ = MatrixXd(4, 4);

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
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

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}

void KalmanFilter::set_State(VectorXd &x_in) {
    x_ = x_in;
}

VectorXd KalmanFilter::get_State(void) {
    return x_;
}

void KalmanFilter::update_StateTransitionMatrix(float deltaTime) {
    /* F_ << 1, 0, deltaTime, 0,
             0, 1, 0, deltaTime,
             0, 0, 1, 0,
             0, 0, 0, 1; */
    F_(0, 2) = deltaTime;
    F_(1, 3) = deltaTime;
}

MatrixXd KalmanFilter::get_StateTransitionMatrix(void) {
    return F_;
}

void KalmanFilter::update_ProcessCovarianceMatrix(float dt, float noise_ax, float noise_ay) {
    /* Avoid duplicating calculations during Q calculation */
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    Q_ <<  ( dt_4/4*noise_ax ),   0                , ( dt_3/2*noise_ax ),   0,
             0                , ( dt_4/4*noise_ay ),   0                , ( dt_3/2*noise_ay ),
           ( dt_3/2*noise_ax ),   0                , ( dt_2*noise_ax )  ,   0,
             0                , ( dt_3/2*noise_ay ),   0                , ( dt_2*noise_ay );
}

MatrixXd KalmanFilter::get_ProcessCovarianceMatrix(void) {
    return Q_;
}

void KalmanFilter::set_StateCovarianceMatrix(MatrixXd P_in) {
    P_ = P_in;
}

MatrixXd KalmanFilter::get_StateCovarianceMatrix(void) {
    return P_;
}

void KalmanFilter::set_MeasurementCovarianceMatrix(MatrixXd R_in) {
    R_ = R_in;
}

MatrixXd KalmanFilter::get_MeasurementCovarianceMatrix(void) {
    return R_;
}

void KalmanFilter::set_MeasurementMatrix(MatrixXd H_in) {
    H_ = H_in;
}

MatrixXd KalmanFilter::get_MeasurementMatrix(void) {
    return H_;
}