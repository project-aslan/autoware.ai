#ifndef _VELOCITY_SET_H
#define _VELOCITY_SET_H

#include <math.h>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "waypoint_follower/libwaypoint_follower.h"

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "velocity_set_info.h"
#include "velocity_set_path.h"
#include <sstream>

class VELOCITY_SET
{
public:
  int RUN(int argc, char** argv);  
};

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

enum class EObstacleType
{
  NONE = -1,
  ON_WAYPOINTS = 1,
  STOPLINE = 3,
};

//////////////////////////////////////
// for visualization of obstacles
//////////////////////////////////////
class ObstaclePoints
{
private:
  std::vector<geometry_msgs::Point> stop_points_;
  std::vector<geometry_msgs::Point> decelerate_points_;
  geometry_msgs::Point previous_detection_;

public:
  void setStopPoint(const geometry_msgs::Point &p)
  {
    stop_points_.push_back(p);
  }
  void setDeceleratePoint(const geometry_msgs::Point &p)
  {
    decelerate_points_.push_back(p);
  }
  geometry_msgs::Point getObstaclePoint(const EControl &kind) const;
  void clearStopPoints()
  {
    stop_points_.clear();
  }
  void clearDeceleratePoints()
  {
    decelerate_points_.clear();
  }

  ObstaclePoints() : stop_points_(0), decelerate_points_(0)
  {
  }
};

inline double calcSquareOfLength(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

// Calculate waypoint index corresponding to distance from begin_waypoint
inline int calcWaypointIndexReverse(const aslan_msgs::Lane &lane, const int begin_waypoint, const double distance)
{
  double dist_sum = 0;
  for (int i = begin_waypoint; i > 0; i--)
  {
    tf::Vector3 v1(lane.waypoints[i].pose.pose.position.x, lane.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(lane.waypoints[i - 1].pose.pose.position.x, lane.waypoints[i - 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);

    if (dist_sum > distance)
      return i;
  }

  // reach the first waypoint
  return 0;
}

namespace velocity_set
{
  constexpr int LOOP_RATE = 10;

  //The distance ahead we search about the deaccelleration radius
  constexpr double DECELERATION_SEARCH_DISTANCE = 20;

  //The distance ahead we search about the stop radius
  constexpr double STOP_SEARCH_DISTANCE = 30;

  

  void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5);
  void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub);
  void displayDetectionRange(const aslan_msgs::Lane& lane, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub);
  int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                       const aslan_msgs::Lane& lane, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       ObstaclePoints* obstacle_points, EObstacleType* obstacle_type,
                       const int wpidx_detection_result_by_other_nodes);
  int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const aslan_msgs::Lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             ObstaclePoints* obstacle_points);
  EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                         const aslan_msgs::Lane& lane, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points, const double points_threshold);
  EControl obstacleDetection(int closest_waypoint, const aslan_msgs::Lane& lane,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacle_pub, int* obstacle_waypoint);
  void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path);
}

#endif /* _VELOCITY_SET_H */
