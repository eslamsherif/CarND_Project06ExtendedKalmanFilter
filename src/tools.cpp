#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   VectorXd rmse(4);
   /* Initialize to some very large values */
   rmse << 1000,1000,1000,1000;

   /* check the validity of the inputs, Sanity Checks */
   if ( ( estimations.size() == 0 ) || ( estimations.size() != ground_truth.size() ) )
   {
      return rmse;
   }

   /* Accumulate squared residuals */
   for(unsigned int idx=0; idx < estimations.size(); ++idx){
      VectorXd temp = estimations[idx] - ground_truth[idx];
      temp = temp.array() * temp.array();
      rmse += temp;
   }

   /* Calculate the mean */
   rmse /= estimations.size();

   /* Calculate the squared root */
   rmse = rmse.array().sqrt();

   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

   MatrixXd Hj(3,4);
   /* Recover state parameters  */

   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   /* Check division by zero */
   if ( (px == 0) || (py == 0) )
   {
      Hj << 0,0,0,0,
            0,0,0,0,
            0,0,0,0;
   }
   else
   {
      float polar_dist = (px*px) + (py*py);
      float sqrt_polar_dist = sqrt(polar_dist);

      Hj << ((px)/(sqrt_polar_dist)),((py)/(sqrt_polar_dist)),(0),(0),
            ((-1*py)/(polar_dist))  ,((px)/(polar_dist))     ,(0),(0),
            ((py*(vx*py-vy*px))/(sqrt_polar_dist*polar_dist)),((px*(vy*px-vx*py))/(sqrt_polar_dist*polar_dist)),((px)/(sqrt_polar_dist)),((py)/(sqrt_polar_dist));
   }

   return Hj;
}
