/*************************************************************************
*   ROS wrapper for continuous homography optical flow 
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
// labrom_optical_flow libraries
#include "labrom_optical_flow/continuous_homography_wrapper.h"

// ROS libraries
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

// ROS messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"

// Eigen libraries
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace optical_flow{
namespace continuous_homography{
//! Continuous homography opt flow node
class Node{
  public:
    //! Constructor
    Node(std::string calibration_file);
    //! Destructor
    ~Node();
    //! Image callback
    void ImageCallback(const sensor_msgs::Image::ConstPtr &msg);
    //! Odometry subscriber 
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    //! Pose subscriber 
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    //! IMU subscriber 
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    //! Publish odometry
    void PublishOdometry(const Eigen::Vector3d &v_camera, const Eigen::Vector3d &w_camera, const std_msgs::Header &header, double dt);
    //! Publish odometry
    void PublishImage(const cv::Mat &image, const std_msgs::Header &header);

  private:
    // ROS handles, publishers and subscribers
    ros::NodeHandle nh_;                       //!< ROS node handle
    ros::NodeHandle pnh_;                       //!< ROS node handle (private)
    image_transport::Subscriber image_sub_;    //!< Input image subscriber
    image_transport::Publisher image_pub_;     //!< Output image publisher
    ros::Subscriber odom_sub_;                 //!< Odometry subscriber (height estimation)
    ros::Subscriber pose_sub_;                 //!< Odometry subscriber (height estimation)
    ros::Subscriber imu_sub_;                   //!< IMU subsccriber (angular speed)
    ros::Publisher odom_pub_;                  //!< Estimated odometry publisher
    tf::TransformListener tf_listener_;        //!< tf listener
 
    // ROS messages
    sensor_msgs::Imu imu_;                      //!< Imu message
    nav_msgs::Odometry opt_flow_odom_;          //!< Output odometry message
 
    // Optical flow related variables
    Wrapper optical_flow_;                          //!< Continuous homogrpahy optical flow
    double height_;                                 //!< Estimated height from feature plane
 
    // Parameters
    double min_frame_rate_in_secs_;                 //!< max time interval between frames arrival (inverse of frame rate)
    int optical_flow_mode_;                         //!< Operation mode (estimate N, w and/or v)                
    double alpha_, beta_;                           //!< Low pass filter parameters      
    double min_height_;                             //!< Minimum allowable camera height
    
    // Miscellaneous
    ros::Time prev_time_;                           //!< last frame timestamp;          

nav_msgs::Odometry odom_;

};
} // continuous_homography namespace
} // optical_flow namespace
