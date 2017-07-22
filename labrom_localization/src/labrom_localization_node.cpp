#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace apriltags_ros
{

class AprilTagFusion
{
  public:
    // Constructor
    AprilTagFusion(void);
    // Destructor
    ~AprilTagFusion(void);
    // AprilTagDetectionArray callback
    void AprilTagDetectionArrayCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr tag_array);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_at_;
    ros::Publisher pub_camera_pose_;
    //
};

/**
 * @brief AprilTagFusion::AprilTagFusion
 */
AprilTagFusion::AprilTagFusion(void)
{
    sub_at_ = nh_.subscribe("/tag_detections", 1, &AprilTagFusion::AprilTagDetectionArrayCallback, this);
    pub_camera_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose", 1);
};

/**
 * @brief AprilTagFusion::AprilTagFusion
 */
AprilTagFusion::~AprilTagFusion(void){};

/**
 * @brief AprilTagFusion::AprilTagDetectionArrayCallback
 * @param tag_array message that contains array of tags
 */
void AprilTagFusion::AprilTagDetectionArrayCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr tag_array)
{
    if (tag_array->detections.size() == 0)
        return;

    static tf::TransformBroadcaster br;

    // tf w_tag
    tf::Transform w_tag;
    w_tag.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion qt;
    qt.setRPY(0, 0, 0);
    w_tag.setRotation(qt);

    tf::Transform c_tag;
    tf::Transform w_camera;
    tf::Vector3 f_pos;
    tf::Quaternion f_qt;
    for (int i = 0; i < tag_array->detections.size(); i++)
    {
        // tf camera
        c_tag.setOrigin(tf::Vector3(
            tag_array->detections[i].pose.pose.position.x,
            tag_array->detections[i].pose.pose.position.y,
            tag_array->detections[i].pose.pose.position.z));
        qt = tf::Quaternion(
            tag_array->detections[i].pose.pose.orientation.x,
            tag_array->detections[i].pose.pose.orientation.y,
            tag_array->detections[i].pose.pose.orientation.z,
            tag_array->detections[i].pose.pose.orientation.w);
        c_tag.setRotation(qt);

        w_camera = w_tag * c_tag.inverse();

        f_pos = w_camera.getOrigin();
        f_qt = w_camera.getRotation();
    }

    geometry_msgs::PoseWithCovarianceStamped camera_pose;
    camera_pose.pose.pose.position.x = f_pos.getX();
    camera_pose.pose.pose.position.y = f_pos.getY();
    camera_pose.pose.pose.position.z = f_pos.getZ();

    camera_pose.pose.pose.orientation.x = qt.x();
    camera_pose.pose.pose.orientation.y = qt.y();
    camera_pose.pose.pose.orientation.z = qt.z();
    camera_pose.pose.pose.orientation.w = qt.w();

    for(int i = 0; i < 6; i++) {
        for(int j = 0; j < 6; j++) {
            if(i == j) {
                camera_pose.pose.covariance[i*6 + j] = 0.01;
            } else {
                camera_pose.pose.covariance[i*6 + j] = 0;
            }
        }
    }

    camera_pose.header.stamp = ros::Time::now();
    camera_pose.header.frame_id = tag_array->detections[0].pose.header.frame_id;

    pub_camera_pose_.publish(camera_pose);
}

} // apriltag_ros namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_fusion_node");

    apriltags_ros::AprilTagFusion atf;

    ros::spin();

    return 0;
};
