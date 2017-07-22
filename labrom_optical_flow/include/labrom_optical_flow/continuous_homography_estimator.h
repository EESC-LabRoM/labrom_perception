/*************************************************************************
*   Class Estimator header files (within the framework of continuous homography)
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

#ifndef OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_ESTIMATOR_H_
#define OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_ESTIMATOR_H_

// labrom_optical_flow libraries
#include "labrom_optical_flow/optical_flow.h"

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/SVD>

// cpp libraries
#include <vector>

namespace optical_flow{
namespace continuous_homography{
//! OF Velocities estimation 
class Estimator{
  public:
    //! Constructor
    Estimator(void);
    //! Destructor
    ~Estimator(void);
    //! Computes matrix A
    //! Estimates Twist 
    void NLinearAngularVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v);
    //! Estimates Linear velocity 
    void NLinearVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, const Eigen::Vector3d &w, Eigen::Vector3d &v);
    //! Estimates Linear velocity (Known plane vector)
    void LinearVelocity(const Eigen::Matrix3d &H, const Eigen::Vector3d &N, const Eigen::Vector3d &w,  Eigen::Vector3d &v);
    //! Estimates camera velocities from feature points and velocities vector 
    bool EstimateCamVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v, int op_mode);

};

} // continuous_homography namespace
} // optical_flow namespace

#endif 