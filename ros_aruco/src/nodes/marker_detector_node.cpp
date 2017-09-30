#include <ros_aruco/marker_detector.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_aruco_marker_detector");
  ros_aruco::MarkerDetector impl;
  impl.configure();
  impl.activate();
  ros::spin();
}
