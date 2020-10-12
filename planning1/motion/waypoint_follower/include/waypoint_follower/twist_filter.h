#include "aslan_msgs/ConfigTwistFilter.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Bool.h"

#include <iostream>

#include "aslan_msgs/ConfigTwistFilter.h"

class twist_filter
{
public: 
	//Publisher
	ros::Publisher g_twist_pub;
	double g_lateral_accel_limit = 10.0;
	double g_lowpass_gain_linear_x = 0.0;
	double g_lowpass_gain_angular_z = 0.0;
	double RADIUS_MAX = 9e10;
	double ERROR = 1e-8;
	int zero_twist_counter = 0;
	std_msgs::Bool obst_removed;
    
    void removedCallback(const std_msgs::Bool::ConstPtr& msg);
    void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void param_callback(const aslan_msgs::ConfigTwistFilter::ConstPtr& config);
    int RUN(int argc, char **argv);
};