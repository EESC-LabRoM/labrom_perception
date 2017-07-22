/*************************************************************************
*   Class Homography implementation for continuous homography framework.
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
#include "labrom_optical_flow/continuous_homography.h"

namespace optical_flow{
namespace continuous_homography{
/**
* Empty constructor.
*/
Homography::Homography(void){};

/**
* Empty destructor.
*/
Homography::~Homography(void){};

/**
* Compute A matrix (see reference by Grabe et al.)
* Matrix A is the stacked matrix of the kronecker product of detected features 
* in camera coordinate and its correspondent skew-symmetric matrix.
* @param x features points vector in camera coordinate.
* @param A output stacked matrix. (no need to pre-allocate memory)
*/
void Homography::ComputeA(const std::vector<Eigen::Vector3d> &x, Eigen::MatrixXd &A){
    // Allocate memory for stacked matrix A
    A = Eigen::MatrixXd(3*x.size(), 9);
    // Allocate memory for temporary result a_i = kronecker(x_i, skew(x_i))
    Eigen::Matrix<double, 3, 9> a_i;
    // Allocate memory for ske-symmetric matrix
    Eigen::Matrix3d skew;
    // Calculate matrix A
    for(int i=0; i<x.size();++i){
        // Assemble skew symmetric matrix
        skew <<     0    ,  -x[i](2) ,  x[i](1),
                 x[i](2) ,     0     , -x[i](0),
                -x[i](1) ,   x[i](0) ,    0     ;

        // Calculate a_i
        a_i.block<3,3>(0,0) << x[i](0)*skew;
        a_i.block<3,3>(0,3) << x[i](1)*skew;
        a_i.block<3,3>(0,6) << x[i](2)*skew;

        // Stack into matrix A
        A.block<3,9>(3*i,0) << a_i; 
    }
}

/**
* Compute B matrix (see reference by Grabe et al.)
* Matrix B is the stacked matrix of the product of ske-symmetric matrix of detected 
* features in camera coordinate (skew(x_i)) and corresponding feature velocity in
* camera coordinate (dx)
* @param x features points vector in camera coordinate.
* @param u feature velocities vector in camera coordinate.
* @param B output stacked matrix (no need to pre-allocate memory)
*/
void Homography::ComputeB(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::MatrixXd &B){
    // Allocate memory for stacked matrix B
    B = Eigen::MatrixXd(3*x.size(), 1);
    // Allocate memory for ske-symmetric matrix
    Eigen::Matrix3d skew;
    // Calculate matrix B
    for(int i=0; i<x.size();++i){
        // Assemble skew symmetric matrix
        skew <<     0    ,  -x[i](2) ,  x[i](1),
                 x[i](2) ,     0     , -x[i](0),
                -x[i](1) ,   x[i](0) ,    0     ;

        // Stack into matrix A
       B.block<3,1>(3*i,0) << skew*dx[i]; 
    }
}

/**
* Compute A and B matrix. Shall be faster than computing independently each matrix (see reference by Grabe et al.). 
* Matrix A is the stacked matrix of the kronecker product of detected features 
* in camera coordinate and its correspondent skew-symmetric matrix.
*
*  Matrix B is the stacked matrix of the product of ske-symmetric matrix of detected 
* features in camera coordinate (skew(x_i)) and corresponding feature velocity in
* camera coordinate (dx)
*
* @param x features points vector in camera coordinate.
* @param u feature velocities vector in camera coordinate.
* @param B output stacked matrix (no need to pre-allocate memory)
* @param A output stacked matrix. (no need to pre-allocate memory)
*/
void Homography::ComputeAandB(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::MatrixXd &A, Eigen::MatrixXd &B){
    // Allocate memory for stacked matrix A and B
    A = Eigen::MatrixXd(3*x.size(), 9);
    B = Eigen::MatrixXd(3*x.size(), 1);
    // Allocate memory for temporary result a_i = kronecker(x_i, skew(x_i))
    Eigen::Matrix<double, 3, 9> a_i;
    // Allocate memory for ske-symmetric matrix
    Eigen::Matrix3d skew;
    // Calculate matrix A
    for(int i=0; i<x.size();++i){
        // Assemble skew symmetric matrix
        skew <<     0    ,  -x[i](2) ,  x[i](1),
                 x[i](2) ,     0     , -x[i](0),
                -x[i](1) ,   x[i](0) ,    0     ;

        // Calculate a_i
        a_i.block<3,3>(0,0) << x[i](0)*skew;
        a_i.block<3,3>(0,3) << x[i](1)*skew;
        a_i.block<3,3>(0,6) << x[i](2)*skew;

        // Stack into matrix A and B
        A.block<3,9>(3*i,0) << a_i; 
        B.block<3,1>(3*i,0) << skew*dx[i]; 
    }
}

/**
* Compute continuous homography matrix (see reference by Grabe et al.). 
* @param x features points vector in camera coordinate.
* @param u feature velocities vector in camera coordinate.
* @param H output continuous homography matrix
*/
void Homography::ComputeHomography(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::Matrix3d &H ){
  // Compute matrix A and B (see definitions in the function description)
  Eigen::MatrixXd A, B;
  ComputeAandB(x, dx, A, B);

  //! @todo see how different tolerance values effect computational cost and performance.
  Eigen::MatrixXd P = optical_flow::continuous_homography::pseudoInverse(A,1.e-4);
  
  // Find h
  Eigen::Matrix<double, 9, 1> h = P*B;

  // Reshape H for matrix
    H << h(0,0), h(3,0), h(6,0), 
         h(1,0), h(4,0), h(7,0), 
         h(2,0), h(5,0), h(8,0);  

         
  //! @todo check if the following approaches have significant better computational cost. PERFORMANCE is worse for sure.
  /* //   Alternative approaches that results were poor:
  // Compute singular value decomposition
  Eigen::JacobiSVD< Eigen::MatrixXd > svd(A ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix<double, 9, 1> h = svd.solve(B);
  // A naive pseudo-inverse computation leads to a low conditioned P matrix
  Eigen::MatrixXd P_naive = ((A.transpose()*A).inverse()) *  A.transpose() ;
  */

}




/**
* Computes pseudo inverse matrix
* @params M matrix to be computed pseudo-inverse
* @params epsilon tolerance error. Choose wisely..
*/
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &M, double epsilon){

  Eigen::JacobiSVD< Eigen::MatrixXd > svd(M ,Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(M.cols(), M.rows())
  *svd.singularValues().array().abs()(0);

  return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(),0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

} // continous_homography namespace
} // optical_flow namespace