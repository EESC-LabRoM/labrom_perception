/*************************************************************************
*   Class Reprojection implementation
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

#include "labrom_optical_flow/reprojection.h"

namespace optical_flow{
/**
* Empty constructor
*/
Reprojection::Reprojection(void){};

/**
* Constructor
* @param fu u-axis focal length
* @param fv v-axis focal length
* @param rho_u u-axis pixel size
* @param rho_v v-axis pixel size
* @param u0 u-axis principal point 
* @param v0 v-axis principal point
*/
//! @todo create a hash table here? Perfomance shall improve... 
Reprojection::Reprojection(double fx, double fy,  double u0, double v0, double rho_u, double rho_v): fx_(fx), 
                                                                                                     fy_(fy),
                                                                                                     u0_(u0),
                                                                                                     v0_(v0),
                                                                                                     rho_u_(rho_u),
                                                                                                     rho_v_(rho_v){};
/**
* Destructor
*/
Reprojection::~Reprojection(void){};

/**
* Reproject points
* @param u image coordinate points vector (in pixels)
* @param x output camera coordinate points vector with normalize Z-axis coordinate.
*/
void Reprojection::ReprojectPoints(const std::vector<cv::Point2f> &u, std::vector<Eigen::Vector3d> &x){
    // Clean output vector
    x.clear();

    // Reproject (frame transformation)
    for(int i=0; i<u.size(); ++i){
      x.push_back(Eigen::Vector3d( (u[i].x-u0_)*rho_u_/fx_,
                                   (u[i].y-v0_)*rho_v_/fy_,
                                   1) );
    }
};

/**
* Reproject velocities
* @param u image coordinate velocity vector (in pixels)
* @param x output camera coordinate velocity vector with normalize Z-axis coordinate.
*/
void Reprojection::ReprojectVelocities(const std::vector<cv::Point2f> &du, std::vector<Eigen::Vector3d> &dx, double dt){
    // Clean output vector
    dx.clear();

    // Coordinate transformation
    for(int i=0; i<du.size(); ++i){
      dx.push_back(Eigen::Vector3d( du[i].x*rho_u_/fx_ / dt,
                                    du[i].y*rho_v_/fy_ / dt,
                                    0) );
    }
};  
}
