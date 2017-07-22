/*************************************************************************
*   Class Continuous Homography Wrapper implementation.
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
#include "labrom_optical_flow/continuous_homography_wrapper.h"

namespace optical_flow{
namespace continuous_homography{
/**
* Empty constructor
*/
Wrapper::Wrapper(void){};

/**
* Constructor
* @param calibration_file name of the file that contains calibration
* @param max_nb_features_to_be_tracked feature detector paremeter
* @param min_nb_features_to_be_tracked feature detector paremeter
*/
Wrapper::Wrapper(std::string calibration_file, int max_nb_features_to_be_tracked, int  min_nb_features_to_be_tracked){
  LK_ = optical_flow::LucasKanade(max_nb_features_to_be_tracked, min_nb_features_to_be_tracked); 
  
  cv::FileStorage fSettings(calibration_file, cv::FileStorage::READ);

  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"]; 
  
  reproject_ = Reprojection(fx,fy,cx,cy,1,1);

};


/**
* Empty destructor
*/
Wrapper::~Wrapper(void){};

/** NLinearVelocity(const Eigen::Matrix3d &H, Eigen::Vector3d &N, const Eigen::Vector3d &w, Eigen::Vector3d &v);
* EStimate velocity from optical flow using continuous homography constraint
* @params input_img grayscale image frame to compute optical flow.
* @param  dt time between image frames
* @param  N vector perpendicular to features plane. It can be either an input or output (see flag estimation mode)
* @param  w camera angular velocity described in the camera frame. It can be either an input or output (see flag estimation mode)
* @param  v camera linear velocity described in the image frame.
*/
int Wrapper::Estimate(cv::Mat &img, Eigen::Vector3d &N, Eigen::Vector3d &w, Eigen::Vector3d &v, double dt, int op_mode){
    static double reset_time = 0;
    int result;

    // Estimate optical flow
    std::vector<cv::Point2f> u, du;
    result = LK_.ComputeOpticalFlow(img, u, du);
    if (result > 0){
      // Reproject feature points and feature velocities
      std::vector<Eigen::Vector3d> x, dx;
      reproject_.ReprojectPoints(u, x);
      reproject_.ReprojectVelocities(du, dx, dt);  
      // Compute continuous homography matrix
      Eigen::Matrix3d H;
      homography_.ComputeHomography(x,dx,H);
      // Estimate velocities
      estimator_.EstimateCamVelocity(H, N, w, v, op_mode);
    }
        
    // If required, create output image.
    if (MODE_SET_OUTPUT_IMAGE & op_mode)
        LK_.DrawOpticalFlow(img, u);    

    return result;
}


} // continuous_homography namespace
} // optical_flow namespace
