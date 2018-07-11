#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#if defined(DEBUG) || defined(DEBUG_PRINTSTATE)
static void PrintKalmanFilterState(KalmanFilter ekf_)
{
    /* IOFormat inspired from https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html */
    const Eigen::IOFormat fmt(4, 0, ",\t", "\n", "\t[", "]");
    // print the output
    cout << "x_ = " << ekf_.get_State().format(fmt)                 << endl;
    cout << endl;
    cout << "P_ = " << ekf_.get_StateCovarianceMatrix().format(fmt) << endl;
    cout << "---------------------------------------------------------" << endl;
}
#endif

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0     , 0,
              0   , 0.0009, 0,
              0   , 0     , 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  MatrixXd P_Init = MatrixXd(4, 4);

  P_Init << 1, 0, 0   , 0,
            0, 1, 0   , 0,
            0, 0, 1000, 0,
            0, 0, 0   , 1000;

  ekf_.Init();
  ekf_.set_StateCovarianceMatrix(P_Init);

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

    VectorXd state(4);

    /* First measurement */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        float Px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
        float Py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
        state << Px, Py, 0, 0;
        ekf_.set_MeasurementCovarianceMatrix(R_radar_);
        ekf_.set_MeasurementMatrix(H_laser_);
        #ifdef DEBUG
        cout << "Initialization with Radar data done." << endl;
        #endif
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        state << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        ekf_.set_MeasurementCovarianceMatrix(R_laser_);
        #ifdef DEBUG
        cout << "Initialization with Laser data done." << endl;
        #endif
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    ekf_.set_State(state);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

   /*****************************************************************************
    *  Prediction
    ****************************************************************************/

    /* Compute the time elapsed between the current and previous measurements */
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;   //dt - expressed in seconds

    ekf_.update_StateTransitionMatrix(dt);
    ekf_.update_ProcessCovarianceMatrix(dt, noise_ax, noise_ay);

    ekf_.Predict();

    #ifdef DEBUG
    cout << "Prediction done." << endl;
    PrintKalmanFilterState(ekf_);
    #endif

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
         * Use the sensor type to perform the update step.
         * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.set_MeasurementCovarianceMatrix(R_radar_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        #ifdef DEBUG
        cout << "Radar Measurement update done." << endl;
        PrintKalmanFilterState(ekf_);
        #endif
    } else {
        // Laser updates
        ekf_.set_MeasurementCovarianceMatrix(R_laser_);
        ekf_.set_MeasurementMatrix(H_laser_);
        ekf_.Update(measurement_pack.raw_measurements_);
        #ifdef DEBUG
        cout << "Laser Measurement update done." << endl;
        PrintKalmanFilterState(ekf_);
        #endif
    }

    #ifdef DEBUG_PRINTSTATE
    PrintKalmanFilterState(ekf_);
    #endif
}
