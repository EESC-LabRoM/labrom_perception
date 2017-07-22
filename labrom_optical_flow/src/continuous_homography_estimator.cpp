/*************************************************************************
*   Class CHEstimator implementation (continuous homography framework).
*   This file is part of labrom_optical_flow
*
*   labrom_optical_flow is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_optical_flow is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_optical_flow.  If not, see <http://www.gnu.org/licenses/>.
*   
*   Reference: V. Grabe, H. H. Bülthoff and P. R. Giordano, "On-board velocity 
*   estimation and closed-loop control of a quadrotor UAV based on optical flow," 
*   Robotics and Automation (ICRA), 2012 IEEE International Conference on, Saint 
*   Paul, MN, 2012, pp. 491-497.
***************************************************************************/

// labrom_optical_flow libraries
#include "labrom_optical_flow/continuous_homography_estimator.h"
#include <iostream>
namespace optical_flow{
namespace continuous_homography{
/**
* Empty constructor
*/
Estimator::Estimator(void){};


/**
* Empty destructor
*/
Estimator::~Estimator(void){};

/**
* Compute linear and angular velocity from continuous homography matrix (see reference by Grabe et al.).
* This function can be called whenever there is no measurement of the linear/angular speed. 
* @param H continuous homography matrix
* @param N unit vector perpendicular to features plane
* @param w angular speed
* @param v linear speed
*/
void Estimator::NLinearAngularVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v){
  // Decompose homography matrix into a skew-symmetric and extract angular speed (w)
  Eigen::Matrix3d H_skew = 0.5*(H - H.transpose());
  w << H_skew(2,1), H_skew(0,2), H_skew(1,0); 

  //  Find corresponding symmetric matrix for computing normalized linear speed (v/d)
  Eigen::Matrix3d H_sym = H-H_skew;
  // Vector N, perpendicular to features planes, has unit norm and spans H_sym. We will find N via singular
  // value decomposition (H_sym = USV'). N corresponds to the first row of V.
  Eigen::JacobiSVD< Eigen::MatrixXd > svd(H_sym ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  N = svd.matrixV().block<3,1>(0,0);

  // Check sign ambiguity considering N_z > 0
  if (N(2) < 0)
    N = -1*N;  

  // Compute normalized linear speed v/d, where d is the features plane depth.
  v = H_sym*N;

  // Correcting signs
  v = -1*v;
  w = -1*w;
}

/**
* Compute linear velocity from continuous homography matrix de-rotating optical flow (see reference by Grabe et al.).
* This function can be called when angular speed measurement is available. 
* @param H continuous homography matrix
* @param N unit vector perpendicular to features plane
* @param w angular speed
* @param v linear speed
*/
void Estimator::NLinearVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, const Eigen::Vector3d &w, Eigen::Vector3d &v){
  // Decompose homography matrix into a skew-symmetric matrix and extract angular speed (w)
  Eigen::Matrix3d H_skew;
  // Notice that skew symmetric matrix is built with sign invertide. w is given in {C}, but the estimation is with respect to feature points plane.
  H_skew <<  0  ,  w(2) , -w(1),
            -w(2),  0   , w(0),
            w(1), -w(0) ,  0   ;

  //  Find corresponding symmetric matrix for computing normalized linear speed (v/d)
  Eigen::Matrix3d H_sym = H-H_skew;
  // Vector N, perpendicular to features planes, has unit norm and spans H_sym. We will find N via singular
  // value decomposition (H_sym = USV'). N corresponds to the first row of V.
  Eigen::JacobiSVD< Eigen::MatrixXd > svd(H_sym ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  N = svd.matrixV().block<3,1>(0,0);
 
  // Check sign ambiguity considering N_z > 0
  if (N(2) > 0)
    N = -1*N;  

  // Compute normalized linear speed v/d, where d is the features plane depth.
  v = H_sym*N;

}

/**
* Compute linear velocity from continuous homography matrix de-rotating optical flow and known feature plane perpendicular vector (see reference by Grabe et al.).
* This function can be called when angular speed measurement is available. 
* @param H continuous homography matrix
* @param N unit vector perpendicular to features plane
* @param w angular speed
* @param v linear speed
*/
void Estimator::LinearVelocity(const Eigen::Matrix3d &H, const Eigen::Vector3d &N, const Eigen::Vector3d &w,  Eigen::Vector3d &v){
  // Decompose homography matrix into a skew-symmetric and extract angular speed (w)
  Eigen::Matrix3d H_skew;
  H_skew <<  0  , -w(2) ,  w(1),
            w(2),  0    , -w(0),
           -w(1),  w(0) ,  0   ;

  //  Find corresponding symmetric matrix for computing normalized linear speed (v/d)
  Eigen::Matrix3d H_sym = H-H_skew;

  // Compute normalized linear speed v/d, where d is the features plane depth.
  v = H_sym*N;

  // Correcting signs
  v = -1*v;

}

/**
*
*/
bool Estimator::EstimateCamVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v, int op_mode){
  int option = (MODE_SET_ESTIMATE_N & op_mode) +  
               (MODE_SET_ESTIMATE_W & op_mode) +
               (MODE_SET_ESTIMATE_V & op_mode);

  switch (option){
    // Neither angular velocity (w) and orthogonal vector to feature plane (N) were given as input
    case (14):
      NLinearAngularVelocity(H, N, w, v);
      break;
    
    // Angular velocity (w) given as input
    case (10):
      NLinearVelocity(H, N, w, v); 
      break;
    
    // Both angular velocity (w) and orthogonal vector to feature plane (N) given as input
    case (8):
      LinearVelocity(H, N, w, v);
      break;

    default:
      NLinearAngularVelocity(H, N, w, v);
      break;
  }

}

} // continuous_homography namespace
} // optical_flow namespace
