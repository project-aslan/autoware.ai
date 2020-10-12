#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <string>

#include "waypoint_follower/libwaypoint_follower.h"
#include "aslan_msgs/LaneArray.h"
#include "aslan_msgs/ConfigLaneStop.h"

enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

typedef std::underlying_type<ChangeFlag>::type ChangeFlagInteger;

class waypoint_marker_publisher
{
public:
	ros::Publisher g_local_mark_pub;
	ros::Publisher g_global_mark_pub;
	std_msgs::ColorRGBA _initial_color;
	std_msgs::ColorRGBA _global_color;
	std_msgs::ColorRGBA g_local_color;
	double g_global_alpha = 0.2;
	double g_local_alpha = 1.0;
	int _closest_waypoint = -1;
	visualization_msgs::MarkerArray g_global_marker_array;
	visualization_msgs::MarkerArray g_local_waypoints_marker_array;
	bool g_config_manual_detection = true;

	void setLifetime(double sec, visualization_msgs::MarkerArray* marker_array);
	void publishMarkerArray(const visualization_msgs::MarkerArray& marker_array, const ros::Publisher& publisher, bool delete_markers=false);
	void createGlobalLaneArrayVelocityMarker(const aslan_msgs::LaneArray& lane_waypoints_array);
	void createGlobalLaneArrayChangeFlagMarker(const aslan_msgs::LaneArray& lane_waypoints_array);
	void createLocalWaypointVelocityMarker(std_msgs::ColorRGBA color, int closest_waypoint, const aslan_msgs::Lane& lane_waypoint);
	void createGlobalLaneArrayMarker(std_msgs::ColorRGBA color, const aslan_msgs::LaneArray& lane_waypoints_array);
	void createGlobalLaneArrayOrientationMarker(const aslan_msgs::LaneArray& lane_waypoints_array);
	void createLocalPathMarker(std_msgs::ColorRGBA color, const aslan_msgs::Lane& lane_waypoint);
	void createLocalPointMarker(const aslan_msgs::Lane& lane_waypoint);
	void configParameter(const aslan_msgs::ConfigLaneStopConstPtr& msg);
	void laneArrayCallback(const aslan_msgs::LaneArrayConstPtr& msg);
	void finalCallback(const aslan_msgs::LaneConstPtr& msg);
	void closestCallback(const std_msgs::Int32ConstPtr& msg);
	int RUN(int argc, char** argv);
};