/*************************************************************************
*   Class Homography header files for continuous homography framework.
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

#ifndef OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_H_
#define OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_H_

// Eigen libraries
#include <Eigen/Core>
#include<Eigen/SVD>

// cpp libraries
#include <vector>

namespace optical_flow{
namespace continuous_homography{
class Homography{
  public:
    //! Constructor
    Homography(void);
    //! Destructor
    ~Homography(void);
    //! Computes matrix A
    void ComputeA(const std::vector<Eigen::Vector3d> &x, Eigen::MatrixXd &A);
    //! Computes matrix B
    void ComputeB(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::MatrixXd &B);
    //! Computes matrix A and B 
    void ComputeAandB(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::MatrixXd &A, Eigen::MatrixXd &B);
    //! Computes Homography 
    void ComputeHomography(const std::vector<Eigen::Vector3d> &x, const std::vector<Eigen::Vector3d> &dx, Eigen::Matrix3d &H);
};

//! Pseudo-inverse function (ref: David link: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257)
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &M, double epsilon = std::numeric_limits<double>::epsilon());

} // continuous_homography namespace
} // optical_flow namespace





#endif // OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_H_
