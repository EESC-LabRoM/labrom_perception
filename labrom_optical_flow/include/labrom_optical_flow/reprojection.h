/*************************************************************************
*   Class Reprojection header files
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
***************************************************************************/

#ifndef OPTICAL_FLOW_REPROJECTION_H_
#define OPTICAL_FLOW_REPROJECTION_H_

// OpenCV libraries
#include "opencv2/imgproc/imgproc.hpp"

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/Dense>

namespace optical_flow{
//! Reproject image points to camera coordinates
class Reprojection{
  public:
    //! Empty constructor
    Reprojection(void);
    //! Constructor
    Reprojection(double fx, double fy, double u0, double v0,  double rho_u=1, double rho_v=1);
    //! Destructor
    ~Reprojection(void);
    //! Reproject points
    void ReprojectPoints(const std::vector<cv::Point2f> &u, std::vector<Eigen::Vector3d> &x);
    //! Reproject velocities
    void ReprojectVelocities(const std::vector<cv::Point2f> &du, std::vector<Eigen::Vector3d> &dx, double dt=1);  

  private:
    double fx_, fy_;              //!< focal lengths
    double rho_u_, rho_v_;        //!< pixel size
    double u0_, v0_;              //!< principal point  
};
}

#endif // OPTICAL_FLOW_REPROJECTION_H_
