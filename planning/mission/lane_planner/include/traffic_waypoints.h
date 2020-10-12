#include <ros/ros.h>

#include "aslan_msgs/ConfigLaneRule.h"
#include "aslan_msgs/LaneArray.h"

class traffic_waypoints
{
public:
	std::string frame_id;

	ros::Publisher traffic_pub;

	aslan_msgs::LaneArray cached_waypoint;

	#ifdef DEBUG
	visualization_msgs::Marker debug_marker;
	ros::Publisher marker_pub;
	int marker_cnt;
	#endif // DEBUG

	aslan_msgs::Lane create_new_lane(const aslan_msgs::Lane& lane, const std_msgs::Header& header);
	#ifdef DEBUG
	std_msgs::ColorRGBA create_color(int index);
	#endif
	void create_waypoint(const aslan_msgs::LaneArray& msg);
	int RUN(int argc, char **argv);
};