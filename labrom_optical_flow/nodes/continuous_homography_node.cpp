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
#include "continuous_homography_node.h"


namespace optical_flow{
namespace continuous_homography{

/**
* Constructor
*/
Node::Node(std::string calibration_file): pnh_("~"){
  // Retrieve data from parameters server
  int max_nb_features_to_be_tracked,  min_nb_features_to_be_tracked;
  bool estimate_v, estimate_w, estimate_N, draw_optical_flow;

  pnh_.param("max_nb_features_to_be_tracked",  max_nb_features_to_be_tracked, 100);
  pnh_.param("min_nb_features_to_be_tracked",  min_nb_features_to_be_tracked, 40);
  pnh_.param("min_frame_rate_in_secs", min_frame_rate_in_secs_, 0.1);
  pnh_.param("draw_optical_flow", draw_optical_flow, false);  
  pnh_.param("initial_height", height_, 0.0);
  pnh_.param("min_height", min_height_, 0.0);
  pnh_.param("estimate_N", estimate_N, true);
  pnh_.param("estimate_w", estimate_w, true);  
  pnh_.param("alpha", alpha_, 1.0);  

  // Optical flow mode flag
  optical_flow_mode_ = MODE_SET_ESTIMATE_V | estimate_N*MODE_SET_ESTIMATE_N | estimate_w*MODE_SET_ESTIMATE_W | draw_optical_flow*MODE_SET_OUTPUT_IMAGE;

  // Low pass filter parameters
  alpha_ = std::min(std::max(alpha_, 0.0), 1.0);
  beta_  = 1 - alpha_;
  
  // Subscribers
  image_transport::ImageTransport it(nh_);
  image_sub_ = it.subscribe("camera/image_raw",1,&Node::ImageCallback, this);
  odom_sub_  = nh_.subscribe("external/odometry",1,&Node::OdomCallback, this);
  pose_sub_  = nh_.subscribe("external/pose",1,&Node::PoseCallback, this);
  imu_sub_   = nh_.subscribe("imu",1,&Node::ImuCallback, this);
  // Publishers
  image_pub_  = it.advertise("optical_flow/image",1);
  odom_pub_   = nh_.advertise<nav_msgs::Odometry>("optical_flow/odometry", 1);
  
  optical_flow_ = Wrapper(calibration_file, max_nb_features_to_be_tracked, min_nb_features_to_be_tracked);
}

/**
* Empty destructor
*/
Node::~Node(void){};

/**
 * IMU subscriber callback
 * @param[in] msg last imu message received
 */
void Node::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg){
	imu_ = *msg;
}

/**
* Update estimated height
* @param[in] msg last odometry message received
*/
void Node::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
  // Updating estimated height
  height_ =  msg->pose.pose.position.z; 
  odom_ = *msg;
}

/**
* Update estimated height
* @param[in] msg last pose message received
*/
void Node::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
  // Transform angular velocity
  height_ =  msg->pose.pose.position.z; 
 }

/**
* Image callback
* Calculates linear speed from consecutives image frames.
* @param[in] msg last image message received 
*/
void Node::ImageCallback(const sensor_msgs::Image::ConstPtr &msg){
  // Check frame rate is fast enough
  double dt =  (msg->header.stamp - prev_time_).toSec();

  prev_time_ = msg->header.stamp;
  if (dt > min_frame_rate_in_secs_){
    ROS_WARN("[optical flow] Frame rate is too slow!");
    return;
  }

  cv_bridge::CvImagePtr cvImagePtr; 
  try{
    // 1: Converting image from ROS to OpenCV format   
    cv::Mat img;
    cvImagePtr     = cv_bridge::toCvCopy(msg);
    img = cvImagePtr->image;
    // 2: Check encoding
    if (msg->encoding != "mono8"){
      if(msg->encoding == "rgb8"){
        cv::cvtColor(img, img, CV_RGB2GRAY);
      } else{
        ROS_ERROR("The image encoding is not proper. Input frame must be in grayscale and encoding set as mono8");
        return;
      }
    }

    // 3: Estimate linear velocity (derotating optical flow)
    Eigen::Vector3d v_camera(0,0,0), w_camera(0,0,0), N_est(0,0,1);
    // 3.1: Transforming angular velocity to camera frame   
    
    if ( !(optical_flow_mode_ & MODE_SET_ESTIMATE_W) ){
      // Transform angular velocity
      try{
        geometry_msgs::Vector3Stamped stamped_imu_w, stamped_camera_w; 
        stamped_imu_w.header = imu_.header;
        stamped_imu_w.vector = imu_.angular_velocity;
        //tf_listener_.transformVector(msg->header.frame_id, stamped_imu_w, stamped_camera_w);
        //w_camera << stamped_camera_w.vector.x, stamped_camera_w.vector.y, stamped_camera_w.vector.z; 
        w_camera << stamped_imu_w.vector.y, stamped_imu_w.vector.x, -stamped_imu_w.vector.z;
      }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
    }
    
    /// 3.2: Estimate optical flow
    int result = optical_flow_.Estimate(img, N_est, w_camera, v_camera, dt, optical_flow_mode_ );

    // 4: If optical flow succeed, send odometry 
    if (result>0) {
      /// Depth scaling 
      v_camera = v_camera*std::fabs(height_); 
      PublishOdometry(v_camera, w_camera, msg->header, dt);
    }

    // 5: Publish image
    if(MODE_SET_OUTPUT_IMAGE & optical_flow_mode_)
      PublishImage(img, msg->header);

    }catch (cv_bridge::Exception &e){
      ROS_ERROR("CV_BRIDGE Exception: %s", e.what());
    }     
}


/**
* Publish image into ROS
* @param[in] image frame to be published
* @param[in] header contains image frame_id and timestamp
*/
void Node::PublishImage(const cv::Mat &image, const std_msgs::Header &header){
  sensor_msgs::ImagePtr opt_flow_image  = cv_bridge::CvImage(header, "mono8", image ).toImageMsg();
  image_pub_.publish(opt_flow_image);
}


/**
* Publish odometry into ROS 
* @param[in] v_camera linear velocity of the camera
* @param[in] w_camera angular velocity of the camera
* @param[in] header description of the message (contains twist frame_id)
*/
void Node::PublishOdometry(const Eigen::Vector3d &v_camera, const Eigen::Vector3d &w_camera, const std_msgs::Header &header, double dt){
  // Assemble odometry

  opt_flow_odom_.header = header;
  opt_flow_odom_.child_frame_id = header.frame_id;
  
  opt_flow_odom_.twist.twist.linear.x  = v_camera(0);//alpha_*v_camera(0) + beta_*opt_flow_odom_.twist.twist.linear.x;
  opt_flow_odom_.twist.twist.linear.y  = v_camera(1);// alpha_*v_camera(1) + beta_*opt_flow_odom_.twist.twist.linear.y;
  opt_flow_odom_.twist.twist.linear.z  = v_camera(2); //alpha_*v_camera(2) + beta_*opt_flow_odom_.twist.twist.linear.z; 
  
  opt_flow_odom_.twist.twist.angular.x = w_camera(0);
  opt_flow_odom_.twist.twist.angular.y = w_camera(1);
  opt_flow_odom_.twist.twist.angular.z = w_camera(2);

// Logging
  geometry_msgs::Quaternion qt = odom_.pose.pose.orientation; 
  geometry_msgs::Point pose = odom_.pose.pose.position;
  geometry_msgs::Vector3 ref = odom_.twist.twist.linear;
  geometry_msgs::Vector3 est = opt_flow_odom_.twist.twist.linear;
  /*
  double yaw   = atan2(2*(qt.w*qt.z + qt.x*qt.y), 1 - 2*(qt.y*qt.y + qt.z*qt.z) );
  std::cout << pose.x << " " << pose.y << " " << pose.z << " " << yaw*180/M_PI << " ";
  std::cout << ref.x << " " << ref.y << " " << ref.z << " ";
  std::cout << -est.y << " " << -est.x << " " << -est.z ;

    std::cout << std::endl;  */
  // Covariance
  opt_flow_odom_.twist.covariance[0] = 0.002;
  opt_flow_odom_.twist.covariance[7] = 0.005;
  opt_flow_odom_.twist.covariance[14] = 0.02;    
  
  // Publish
  odom_pub_.publish( opt_flow_odom_);

 }

} // continuous_homography namespace
} // optical_flow namespace

int main(int argc, char** argv){
    ros::init(argc,argv,"OpticalFlow");

    if(argc != 2){
      ROS_ERROR("Wrong usage! \nTry: rosrun labrom_mav_optical_flow node PATH_TO_CALIBRATION");
      return -1;
    }
    optical_flow::continuous_homography::Node node(argv[1]);

    ros::spin();
    
    return 0;
}

