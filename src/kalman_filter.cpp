#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#ifdef DEBUG_EKF
#include <iostream>
#include <string>

using namespace std;

static void PrintMatrix(string str, MatrixXd prnt)
{
    /* IOFormat inspired from https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html */
    const Eigen::IOFormat fmt(4, 0, ",\t", "\n", "\t[", "]");
    // print the output
    cout << str << prnt.format(fmt) << endl;
    cout << endl;
}
#endif

/* Private Functions */

static float CorrectAnglePhi(float Phi)
{
    while ( Phi > M_PI || Phi < -M_PI )
    {
        if ( Phi > M_PI )
        {
            Phi -= M_PI;
        }
        else if (Phi < -M_PI)
        {
            Phi += M_PI;
        }
    }

    return Phi;
}

/* Convert Cartesian to Polar coordinates */
static VectorXd C2P(const VectorXd& state)
{
    float px = state(0);
    float py = state(1);
    float vx = state(2);
    float vy = state(3);

    float RootofSquaredSums = sqrt( ( px * px ) + ( py * py ) );

    float Range     = RootofSquaredSums;
    float Bearing   = atan2(py , px);
    float RangeRate;
    if(RootofSquaredSums < EPSI)
    {
        RangeRate = 0.0f;
    }
    else
    {
        RangeRate = ( ( ( px * vx ) + ( py * vy ) ) / ( RootofSquaredSums ) );
    }

#ifdef DEBUG_EKF
    cout << "px " << px << endl;
    cout << "py " << py << endl;
    cout << "vx " << vx << endl;
    cout << "vy " << vy << endl;
    cout << "RootofSquaredSums " << RootofSquaredSums << endl;
    cout << "Range " << Range << endl;
    cout << "Bearing " << Bearing << endl;
    cout << "RangeRate " << RangeRate << endl;
#endif

    VectorXd out(3);
    out << Range, Bearing, RangeRate;
    return out;
}

static MatrixXd CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj(3,4);

    /* Recover state parameters  */
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float polar_dist = (px*px) + (py*py);
    float sqrt_polar_dist = sqrt(polar_dist);
    float mul = sqrt_polar_dist*polar_dist;

    if(polar_dist < EPSI)
    {
        return Hj;
    }

    Hj <<   ((px)/(sqrt_polar_dist))   ,((py)/(sqrt_polar_dist))   ,(0)                      ,(0),
            ((-1*py)/(polar_dist))     ,((px)/(polar_dist))        ,(0)                      ,(0),
            ((py*(vx*py-vy*px))/(mul)) ,((px*(vy*px-vx*py))/(mul)) ,((px)/(sqrt_polar_dist)) ,((py)/(sqrt_polar_dist));

    return Hj;
}

/* Public Functions */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init()
{
    x_  = VectorXd(4);
    P_  = MatrixXd(4, 4);
    F_  = MatrixXd(4, 4);
    H_  = MatrixXd(2, 4);
    R_  = MatrixXd(2, 2);
    Q_  = MatrixXd(4, 4);
    Hj_ = MatrixXd(3, 4);
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    Init();

    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
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

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    VectorXd z_pred = C2P(x_);
    VectorXd y = z - z_pred;
    y(1) = CorrectAnglePhi(y(1));
    Hj_ = CalculateJacobian(x_);
    MatrixXd Hjt = Hj_.transpose();
    MatrixXd S = Hj_ * P_ * Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHjt = P_ * Hjt;
    MatrixXd K = PHjt * Si;

    #ifdef DEBUG_EKF
    PrintMatrix("z_pred", z_pred);
    PrintMatrix("y", y);
    PrintMatrix("Hj_", Hj_);
    PrintMatrix("Hjt", Hjt);
    PrintMatrix("S", S);
    PrintMatrix("Si", Si);
    PrintMatrix("PHjt", PHjt);
    PrintMatrix("K", K);
    #endif

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
}

void KalmanFilter::set_State(VectorXd &x_in)
{
    x_ = x_in;
}

VectorXd KalmanFilter::get_State(void)
{
    return x_;
}

void KalmanFilter::update_StateTransitionMatrix(float deltaTime)
{
    F_ << 1, 0, deltaTime, 0,
          0, 1, 0        , deltaTime,
          0, 0, 1        , 0,
          0, 0, 0        , 1;
    // F_(0, 2) = deltaTime;
    // F_(1, 3) = deltaTime;
}

MatrixXd KalmanFilter::get_StateTransitionMatrix(void)
{
    return F_;
}

void KalmanFilter::update_ProcessCovarianceMatrix(float dt, float noise_ax, float noise_ay)
{
    /* Avoid duplicating calculations during Q calculation */
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_3_2 = dt_3/2.0f;
    float dt_4 = dt_3 * dt;
    float dt_4_4 = dt_4/4.0f;

    Q_ <<  ( dt_4_4*noise_ax ),   0                , ( dt_3_2*noise_ax ),   0,
             0                , ( dt_4_4*noise_ay ),   0                , ( dt_3_2*noise_ay ),
           ( dt_3_2*noise_ax ),   0                , ( dt_2*noise_ax )  ,   0,
             0                , ( dt_3_2*noise_ay ),   0                , ( dt_2*noise_ay );
}

MatrixXd KalmanFilter::get_ProcessCovarianceMatrix(void)
{
    return Q_;
}

void KalmanFilter::set_StateCovarianceMatrix(MatrixXd P_in)
{
    P_ = P_in;
}

MatrixXd KalmanFilter::get_StateCovarianceMatrix(void)
{
    return P_;
}

void KalmanFilter::set_MeasurementCovarianceMatrix(MatrixXd R_in)
{
    R_ = R_in;
}

MatrixXd KalmanFilter::get_MeasurementCovarianceMatrix(void)
{
    return R_;
}

void KalmanFilter::set_MeasurementMatrix(MatrixXd H_in)
{
    H_ = H_in;
}

MatrixXd KalmanFilter::get_MeasurementMatrix(void)
{
    return H_;
}