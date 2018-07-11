#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
private:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);
  void Init();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Setter and Getter functions
   */
  Eigen::VectorXd get_State(void);
  void set_State(Eigen::VectorXd &x_in);

  Eigen::MatrixXd get_StateTransitionMatrix(void);
  void update_StateTransitionMatrix(float deltaTime);

  Eigen::MatrixXd get_ProcessCovarianceMatrix(void);
  void update_ProcessCovarianceMatrix(float deltaTime, float nax, float nay);

  Eigen::MatrixXd get_StateCovarianceMatrix(void);
  void set_StateCovarianceMatrix(Eigen::MatrixXd P_in);

  Eigen::MatrixXd get_MeasurementCovarianceMatrix(void);
  void set_MeasurementCovarianceMatrix(Eigen::MatrixXd P_in);

  Eigen::MatrixXd get_MeasurementMatrix(void);
  void set_MeasurementMatrix(Eigen::MatrixXd H_in);
};

#endif /* KALMAN_FILTER_H_ */
