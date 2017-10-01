#ifndef __ROS_ARUCO_MARKER_CONFIG_H__
#define __ROS_ARUCO_MARKER_CONFIG_H__

#include <ros/ros.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

namespace ros_aruco
{
  struct Group
  {
    enum EstimationType
    {
      TYPE_GRID=0, //! estimate pose of a marker board
      TYPE_CHARUCO, //! estimate pose of a charuco board
      TYPE_CHARUCO_CORNERS, //! charuco but no pose, only corners (for calibration)
    };

    EstimationType type;
    int start;
    int num_x;
    int num_y;
    double size;
    double spacing;
    cv::Ptr<cv::aruco::Board> board;

    static std::unique_ptr<Group> create(
      const ros::NodeHandle& nh,
      const cv::Ptr<cv::aruco::Dictionary>& dict)
    {
      std::unique_ptr<Group> g(new Group);

      // Read parameters from server
      int type;
      nh.param<int>   ("type",    type, 0);
      nh.param<int>   ("start",   g->start, 0);
      nh.param<int>   ("num_x",   g->num_x, 5);
      nh.param<int>   ("num_y",   g->num_y, 7);
      nh.param<double>("size",    g->size, 0.05);
      nh.param<double>("spacing", g->spacing, 0.01);
      g->type = static_cast<EstimationType>(type);

      // Generate board and dictionary
      switch(g->type)
      {
      case(TYPE_GRID):
      {
        g->board = cv::aruco::GridBoard::create(
          g->num_x, g->num_y, g->size, g->spacing, dict, g->start);
        break;
      }
      case(TYPE_CHARUCO):
      case(TYPE_CHARUCO_CORNERS):
      {
        g->board = cv::aruco::CharucoBoard::create(
          g->num_x, g->num_y, g->size, g->size-2.*g->spacing, dict);
        for (auto&& id : g->board->ids) { id += g->start; }
        break;
      }
      }

      return g;
    }

  };

  struct MarkerConfig
  {
    int bits;
    int num_ids;
    double size;

    cv::Ptr<cv::aruco::Dictionary> dict;
    std::vector<std::unique_ptr<Group> > groups;

    static std::unique_ptr<MarkerConfig> create(const ros::NodeHandle& nh)
    {
      std::unique_ptr<MarkerConfig> c(new MarkerConfig);

      // Read parameters
      std::vector<std::string> groups;
      nh.param<int>("bits",    c->bits, 6);
      nh.param<int>("num_ids", c->num_ids, 50);
      nh.param<std::vector<std::string> >("groups", groups, std::vector<std::string>());
      c->dict = cv::aruco::generateCustomDictionary(c->num_ids, c->bits);
      c->groups.resize(groups.size());
      for (size_t i=0; i<groups.size(); ++i)
      {
        c->groups[i] = Group::create(ros::NodeHandle(nh, groups[i]), c->dict);
      }

      return c;
    }
  };
}

#endif
