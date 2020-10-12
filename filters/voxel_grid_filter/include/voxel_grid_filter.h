#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "aslan_msgs/ConfigVoxelGridFilter.h"

#include <voxel_grid_filter/PointsDownsamplerInfo.h>

#include <chrono>

#include "points_downsampler.h"

#define MAX_MEASUREMENT_RANGE 200.0

class voxel_grid_class
{
public:
	ros::Publisher filtered_points_pub;

	// Leaf size of VoxelGrid filter.
	double voxel_leaf_size = 2.0;

	ros::Publisher points_downsampler_info_pub;
	voxel_grid_filter::PointsDownsamplerInfo points_downsampler_info_msg;

	std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;

	bool _output_log = false;
	std::ofstream ofs;
	std::string filename;

	std::string POINTS_TOPIC;
	double measurement_range = MAX_MEASUREMENT_RANGE;

	void param_callback(const aslan_msgs::ConfigVoxelGridFilter::ConstPtr& input);
	void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
	int RUN(int argc, char** argv);
};