#include <ros/ros.h>
#include <ros_aruco_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/aruco.hpp>

namespace ros_aruco
{
class MarkerDetector
{
public:
  MarkerDetector(const ros::NodeHandle& nh=ros::NodeHandle(),
                 const ros::NodeHandle& nh_private=ros::NodeHandle("~"))
    : active_(false)
    , nh_(nh)
    , nh_priv_(nh_private)
  {}

  bool configure()
  {
    nh_priv_.param<int>("marker_bits", marker_bits_, 6);
    nh_priv_.param<int>("marker_max_id_", marker_max_id_, 50);
    nh_priv_.param<bool>("publish_debug_image", publish_debug_image_, false);

    it_.reset( new image_transport::ImageTransport(nh_) );
    sub_img_ = it_->subscribeCamera("image", 5, &MarkerDetector::imageCb, this);

    tf_buffer_.reset( new tf2_ros::Buffer );
    tf_listener_.reset( new tf2_ros::TransformListener(*tf_buffer_) );

    if (publish_debug_image_) { pub_debug_image_ = it_->advertise("image_debug", 1); }

    return true;
  }

  bool cleanup()
  {
    if (publish_debug_image_) { pub_debug_image_.shutdown(); }

    tf_listener_.reset();
    tf_buffer_.reset();

    sub_img_.shutdown();
    it_.reset();

    return true;
  }

  bool activate() { return active_ = true; }
  bool deactivate() { return !(active_ = false); }

  void imageCb(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (!active_) return;

    camera_model_.fromCameraInfo(info_msg);
    const cv::Mat img = cv_bridge::toCvShare(img_msg)->image;

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::generateCustomDictionary(marker_max_id_, marker_bits_);
    cv::aruco::detectMarkers(img, dictionary, corners, ids, parameters);

    if (publish_debug_image_)
    {
      cv::Mat img_debug;
      img.copyTo(img_debug);
      cv::aruco::drawDetectedMarkers(img_debug, corners, ids);
      pub_debug_image_.publish(
        cv_bridge::CvImage(
          img_msg->header, img_msg->encoding, img_debug).toImageMsg());
    }
  }

private:
  bool active_, publish_debug_image_;
  int marker_max_id_, marker_bits_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_img_;
  image_geometry::PinholeCameraModel camera_model_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  ros::Publisher pub_pose_;
  ros::Publisher pub_marker_;
  image_transport::Publisher pub_debug_image_;

};
}
