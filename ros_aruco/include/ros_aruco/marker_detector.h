#ifndef __ROS_ARUCO_MARKER_DETECTOR_H__
#define __ROS_ARUCO_MARKER_DETECTOR_H__

#include <ros/ros.h>
#include <ros_aruco_msgs/Corner.h>
#include <ros_aruco_msgs/Marker.h>
#include <ros_aruco_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/aruco.hpp>

#include <ros_aruco/marker_config.h>

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

  bool activate() { return active_ = true; }
  bool deactivate() { return !(active_ = false); }

  bool configure();
  bool cleanup();

  void imageCb(const sensor_msgs::ImageConstPtr& img_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

private:
  bool active_, publish_debug_image_;
  std::unique_ptr<MarkerConfig> cfg_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber sub_img_;
  image_geometry::PinholeCameraModel camera_model_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  ros::Publisher pub_pose_;
  ros::Publisher pub_markers_;
  image_transport::Publisher pub_debug_image_;

  cv::Ptr<cv::aruco::DetectorParameters> params_;

  std::vector<cv::Vec3d> rvecs_, tvecs_;
};

void toTF(const cv::Vec3d& rvec, const cv::Vec3d& tvec, tf2::Transform& tf)
{
  cv::Mat_<double> rmat;
  cv::Rodrigues(rvec, rmat);
  tf = tf2::Transform(
    tf2::Matrix3x3(
      rmat(0,0), rmat(0,1), rmat(0,2),
      rmat(1,0), rmat(1,1), rmat(1,2),
      rmat(2,0), rmat(2,1), rmat(2,2)),
    tf2::Vector3(tvec(0), tvec(1), tvec(2)));
}

bool MarkerDetector::configure()
{
  nh_priv_.param<bool>("publish_debug_image", publish_debug_image_, false);

  cfg_ = MarkerConfig::create(ros::NodeHandle(nh_priv_, "marker"));
  params_ = cv::aruco::DetectorParameters::create(); // There are lot.

  it_.reset( new image_transport::ImageTransport(nh_) );
  sub_img_ = it_->subscribeCamera("image_raw", 5, &MarkerDetector::imageCb, this);

  tf_buffer_.reset( new tf2_ros::Buffer );
  tf_listener_.reset( new tf2_ros::TransformListener(*tf_buffer_) );

  pub_markers_ = nh_.advertise<ros_aruco_msgs::MarkerArray>("detections", 1);

  if (publish_debug_image_) { pub_debug_image_ = it_->advertise("image_debug", 1); }

  return true;
}

bool MarkerDetector::cleanup()
{
  if (publish_debug_image_) { pub_debug_image_.shutdown(); }
  pub_markers_.shutdown();

  tf_listener_.reset();
  tf_buffer_.reset();

  sub_img_.shutdown();
  it_.reset();

  cfg_.reset();

  return true;
}

void MarkerDetector::imageCb(const sensor_msgs::ImageConstPtr& img_msg,
                             const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (!active_) return;

  camera_model_.fromCameraInfo(info_msg);
  const cv::Mat img = cv_bridge::toCvShare(img_msg)->image;

  double confidence; // number of detected ids / number of expected ids
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners;
  cv::aruco::detectMarkers(img, cfg_->dict, corners, ids, params_);

  // if we couln't find any there is nothing to do anymore
  if (ids.size() == 0) { return; }

  ros_aruco_msgs::MarkerArray ma;
  ma.header = img_msg->header;
  ma.markers.resize(ids.size());
  for (size_t i=0; i<ids.size(); ++i)
  {
    ma.markers[i].header = img_msg->header;
    ma.markers[i].id = ids[i];
    ma.markers[i].corners.resize(corners[i].size());
    for (size_t j=0; j<corners[i].size(); ++j)
    {
      ma.markers[i].corners[j].x = corners[i][j].x;
      ma.markers[i].corners[j].y = corners[i][j].y;
    }
  }
  pub_markers_.publish(ma);

  if (cfg_->groups.size() == 0)
  {
    cv::aruco::estimatePoseSingleMarkers(
      corners, cfg_->size,
      camera_model_.intrinsicMatrix(),
      camera_model_.distortionCoeffs(),
      rvecs_, tvecs_);
    confidence = 1.;
  }
  else
  {
    rvecs_.resize(cfg_->groups.size());
    tvecs_.resize(cfg_->groups.size());

    for (size_t i=0; i<cfg_->groups.size(); ++i)
    {
      switch(cfg_->groups[i]->type)
      {
      case(Group::TYPE_GRID):
      {
        confidence = cv::aruco::estimatePoseBoard(
          corners, ids, cfg_->groups[i]->board,
          camera_model_.intrinsicMatrix(),
          camera_model_.distortionCoeffs(),
          rvecs_[i], tvecs_[i]) / double(cfg_->groups[i]->board->ids.size());
        break;
      }
      case(Group::TYPE_CHARUCO):
      case(Group::TYPE_CHARUCO_CORNERS):
      {
        std::vector<cv::Point2f> charuco_corners;
        std::vector<int> charuco_ids;
        cv::aruco::interpolateCornersCharuco(
          corners, ids, img,
          static_cast<cv::aruco::CharucoBoard*>(&(*cfg_->groups[i]->board)),
          charuco_corners,
          charuco_ids,
          camera_model_.intrinsicMatrix(),
          camera_model_.distortionCoeffs());
        if(cfg_->groups[i]->type==Group::TYPE_CHARUCO)
        {
          cv::aruco::estimatePoseCharucoBoard(
            charuco_corners, charuco_ids,
            static_cast<cv::aruco::CharucoBoard*>(&(*cfg_->groups[i]->board)),
            camera_model_.intrinsicMatrix(),
            camera_model_.distortionCoeffs(),
            rvecs_[i], tvecs_[i]);
        }
        break;
      }
      }
    }
  }

  for (size_t i=0; i<rvecs_.size(); ++i)
  {
    tf2::Transform tf;
    toTF(rvecs_[i], tvecs_[i], tf);
  }

  if (publish_debug_image_)
  {
    cv::Mat img_debug;
    img.copyTo(img_debug);
    cv::aruco::drawDetectedMarkers(img_debug, corners, ids);
    for (size_t i=0; i<rvecs_.size(); ++i)
    {
      cv::aruco::drawAxis(
        img_debug,
        camera_model_.intrinsicMatrix(),
        camera_model_.distortionCoeffs(),
        rvecs_[i], tvecs_[i], 0.1);
    }

    pub_debug_image_.publish(
      cv_bridge::CvImage(
        img_msg->header, img_msg->encoding, img_debug).toImageMsg());
  }
}

}

#endif
