/*
 * Originally included at Autoware.ai version 1.10.0 and
 * has been modified and redesigned significantly for Project Aslan
 *
 * Author: Abdelrahman Barghouth
 *
 * Copyright (C) 2020 Project ASLAN - All rights reserved
 *
 * Original copyright notice:
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "aslan_msgs/ConfigVoxelGridFilter.h"

#include <voxel_grid_filter/PointsDownsamplerInfo.h>

#include <chrono>

#include "points_downsampler.h"
#include "voxel_grid_filter.h"

void voxel_grid_class::param_callback(const aslan_msgs::ConfigVoxelGridFilter::ConstPtr& input)
{
  voxel_grid_class::voxel_leaf_size = input->voxel_leaf_size;
  voxel_grid_class::measurement_range = input->measurement_range;

}

void voxel_grid_class::scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);

  if(voxel_grid_class::measurement_range != MAX_MEASUREMENT_RANGE){
    scan = removePointsByRange(scan, 0, voxel_grid_class::measurement_range);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  sensor_msgs::PointCloud2 filtered_msg;

  voxel_grid_class::filter_start = std::chrono::system_clock::now();

  // if voxel_grid_class::voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (voxel_grid_class::voxel_leaf_size >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_grid_class::voxel_leaf_size, voxel_grid_class::voxel_leaf_size, voxel_grid_class::voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  }
  else
  {
    pcl::toROSMsg(*scan_ptr, filtered_msg);
  }

  voxel_grid_class::filter_end = std::chrono::system_clock::now();

  filtered_msg.header = input->header;
  voxel_grid_class::filtered_points_pub.publish(filtered_msg);

  voxel_grid_class::points_downsampler_info_msg.header = input->header;
  voxel_grid_class::points_downsampler_info_msg.filter_name = "voxel_grid_filter";
  voxel_grid_class::points_downsampler_info_msg.measurement_range = voxel_grid_class::measurement_range;
  voxel_grid_class::points_downsampler_info_msg.original_points_size = scan.size();
  if (voxel_grid_class::voxel_leaf_size >= 0.1)
  {
    voxel_grid_class::points_downsampler_info_msg.filtered_points_size = filtered_scan_ptr->size();
  }
  else
  {
    voxel_grid_class::points_downsampler_info_msg.filtered_points_size = scan_ptr->size();
  }
  voxel_grid_class::points_downsampler_info_msg.original_ring_size = 0;
  voxel_grid_class::points_downsampler_info_msg.filtered_ring_size = 0;
  voxel_grid_class::points_downsampler_info_msg.exe_time = std::chrono::duration_cast<std::chrono::microseconds>(voxel_grid_class::filter_end - voxel_grid_class::filter_start).count() / 1000.0;
  voxel_grid_class::points_downsampler_info_pub.publish(voxel_grid_class::points_downsampler_info_msg);

  if(voxel_grid_class::_output_log == true){
	  if(!voxel_grid_class::ofs){
		  std::cerr << "Could not open " << voxel_grid_class::filename << "." << std::endl;
		  exit(1);
	  }
	  voxel_grid_class::ofs << voxel_grid_class::points_downsampler_info_msg.header.seq << ","
		  << voxel_grid_class::points_downsampler_info_msg.header.stamp << ","
		  << voxel_grid_class::points_downsampler_info_msg.header.frame_id << ","
		  << voxel_grid_class::points_downsampler_info_msg.filter_name << ","
		  << voxel_grid_class::points_downsampler_info_msg.original_points_size << ","
		  << voxel_grid_class::points_downsampler_info_msg.filtered_points_size << ","
		  << voxel_grid_class::points_downsampler_info_msg.original_ring_size << ","
		  << voxel_grid_class::points_downsampler_info_msg.filtered_ring_size << ","
		  << voxel_grid_class::points_downsampler_info_msg.exe_time << ","
		  << std::endl;
  }

}

int voxel_grid_class::RUN(int argc, char** argv)
{
  ros::init(argc, argv, "voxel_grid_filter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.getParam("points_topic", voxel_grid_class::POINTS_TOPIC);
  private_nh.getParam("output_log", voxel_grid_class::_output_log);

  if(voxel_grid_class::_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  voxel_grid_class::filename = "voxel_grid_filter_" + std::string(buffer) + ".csv";
	  voxel_grid_class::ofs.open(voxel_grid_class::filename.c_str(), std::ios::app);
  }

  // Publishers
  voxel_grid_class::filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);
  voxel_grid_class::points_downsampler_info_pub = nh.advertise<voxel_grid_filter::PointsDownsamplerInfo>("/voxel_grid_filter_info", 1000);

  // Subscribers
  ros::Subscriber param_sub = nh.subscribe("config/voxel_grid_filter", 10, &voxel_grid_class::param_callback, this);
  ros::Subscriber scan_sub = nh.subscribe(voxel_grid_class::POINTS_TOPIC, 10, &voxel_grid_class::scan_callback, this);

  ros::spin();

  return 0;
}
