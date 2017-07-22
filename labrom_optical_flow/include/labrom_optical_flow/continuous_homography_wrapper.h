/*************************************************************************
*   Class Continuous Homography Wrapper header files.
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

#ifndef OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_WRAPPER_H_
#define OPTICAL_FLOW_CONTINUOUS_HOMOGRAPHY_WRAPPER_H_

// labrom_optical_flow libraries
#include "labrom_optical_flow/lucas_kanade.h"
#include "labrom_optical_flow/reprojection.h"
#include "labrom_optical_flow/continuous_homography.h"
#include "labrom_optical_flow/continuous_homography_estimator.h"

// OpenCV libraries
#include "opencv2/core/core.hpp"
// Eigen libraries
#include <Eigen/Core>

namespace optical_flow{
namespace continuous_homography{
class Wrapper{
  public:
    //! Empty constructor
    Wrapper(void);
    //! Constructor
    Wrapper(std::string calibration_file, int max_nb_features_to_be_tracked, int  min_nb_features_to_be_tracked);
    //! Destructor
    ~Wrapper(void);
    //! Estimate velocities from input image
    int Estimate(cv::Mat &img, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v, double dt, int op_mode);

  private:
    optical_flow::LucasKanade LK_;
    optical_flow::Reprojection reproject_;
    optical_flow::continuous_homography::Homography homography_;
    optical_flow::continuous_homography::Estimator estimator_;
}; 
 
} // continuous_homography namesoace
} // optical_flow namespace

#endif
