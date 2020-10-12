#include <condition_variable>
#include <queue>
#include <thread>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include "aslan_msgs/LaneArray.h"

#include <map_file/get_file.h>

struct Area {
	std::string path;
	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;
};

typedef std::vector<Area> AreaList;
typedef std::vector<std::vector<std::string>> Tbl;

class RequestQueue {
private:
	std::queue<geometry_msgs::Point> queue_; // takes priority over look_ahead_queue_
	std::queue<geometry_msgs::Point> look_ahead_queue_;
	std::mutex mtx_;
	std::condition_variable cv_;

public:
	void enqueue(const geometry_msgs::Point& p);
	void enqueue_look_ahead(const geometry_msgs::Point& p);
	void clear();
	void clear_look_ahead();
	geometry_msgs::Point dequeue();
};

class pcd_loader
{
public:
	int DEFAULT_UPDATE_RATE = 1000; // ms
	double MARGIN_UNIT = 100; // meter
	int ROUNDING_UNIT = 1000; // meter
	std::string AREALIST_FILENAME = "arealist.txt";
	std::string TEMPORARY_DIRNAME = "/tmp/";

	int update_rate;
	int fallback_rate;
	double margin;
	bool can_download;

	ros::Time gnss_time;
	ros::Time current_time;

	ros::Publisher pcd_pub;
	ros::Publisher stat_pub;
	std_msgs::Bool stat_msg;

	AreaList all_areas;
	AreaList downloaded_areas;
	std::mutex downloaded_areas_mtx;
	std::vector<std::string> cached_arealist_paths;

	GetFile gf;
	RequestQueue request_queue;

	Tbl read_csv(const std::string& path);
	void write_csv(const std::string& path, const Tbl& tbl);
	AreaList read_arealist(const std::string& path);
	void write_arealist(const std::string& path, const AreaList& areas);
	bool is_downloaded(const std::string& path);
	bool is_in_area(double x, double y, const Area& area, double m);
	std::string create_location(int x, int y);
	void cache_arealist(const Area& area, AreaList& areas);
	int download(GetFile gf, const std::string& tmp, const std::string& loc, const std::string& filename);
	void download_map();
	sensor_msgs::PointCloud2 create_pcd(const geometry_msgs::Point& p);
	sensor_msgs::PointCloud2 create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err = NULL);
	void publish_pcd(sensor_msgs::PointCloud2 pcd, const int* errp = NULL);
	void publish_gnss_pcd(const geometry_msgs::PoseStamped& msg);
	void publish_current_pcd(const geometry_msgs::PoseStamped& msg);
	void publish_dragged_pcd(const geometry_msgs::PoseWithCovarianceStamped& msg);
	void request_lookahead_download(const aslan_msgs::LaneArray& msg);
	void print_usage();
	int RUN(int argc, char **argv);

};
