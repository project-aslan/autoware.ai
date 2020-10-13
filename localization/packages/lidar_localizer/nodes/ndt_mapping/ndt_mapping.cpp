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

/*
 Localization and mapping program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <aslan_msgs/ConfigNDTMapping.h>
#include <aslan_msgs/ConfigNDTMappingOutput.h>

#include <time.h>
#include "ndt_mapping.h"

void ndt_mapping::param_callback(const aslan_msgs::ConfigNDTMapping::ConstPtr& input)
{
    ndt_mapping::ndt_res = input->resolution;
    ndt_mapping::step_size = input->step_size;
    ndt_mapping::trans_eps = input->trans_epsilon;
    ndt_mapping::max_iter = input->max_iterations;
    ndt_mapping::voxel_leaf_size = input->leaf_size;
    ndt_mapping::min_scan_range = input->min_scan_range;
    ndt_mapping::max_scan_range = input->max_scan_range;
    min_add_scan_shift = input->min_add_scan_shift;

    std::cout << "ndt_mapping::param_callback ndt_mapping" << std::endl;
    std::cout << "ndt_mapping::ndt_res: " << ndt_mapping::ndt_res << std::endl;
    std::cout << "ndt_mapping::step_size: " << ndt_mapping::step_size << std::endl;
    std::cout << "ndt_mapping::trans_epsilon: " << ndt_mapping::trans_eps << std::endl;
    std::cout << "ndt_mapping::max_iter: " << ndt_mapping::max_iter << std::endl;
    std::cout << "ndt_mapping::voxel_leaf_size: " << ndt_mapping::voxel_leaf_size << std::endl;
    std::cout << "ndt_mapping::min_scan_range: " << ndt_mapping::min_scan_range << std::endl;
    std::cout << "ndt_mapping::max_scan_range: " << ndt_mapping::max_scan_range << std::endl;
    std::cout << "min_add_scan_shift: " << min_add_scan_shift << std::endl;
}

void ndt_mapping::output_callback(const aslan_msgs::ConfigNDTMappingOutput::ConstPtr& input)
{
    double filter_res = input->filter_res;
    std::string filename = input->filename;
    std::cout << "ndt_mapping::output_callback" << std::endl;
    std::cout << "filter_res: " << filter_res << std::endl;
    std::cout << "filename: " << filename << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    map_ptr->header.frame_id = "map";
    map_filtered->header.frame_id = "map";
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);

    // Apply voxelgrid filter
    if (filter_res == 0.0)
    {
        std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
        pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    }
    else
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(filter_res, filter_res, filter_res);
        voxel_grid_filter.setInputCloud(map_ptr);
        voxel_grid_filter.filter(*map_filtered);
        std::cout << "Original: " << map_ptr->points.size() << " points." << std::endl;
        std::cout << "Filtered: " << map_filtered->points.size() << " points." << std::endl;
        pcl::toROSMsg(*map_filtered, *map_msg_ptr);
    }

    ndt_mapping::ndt_map_pub.publish(*map_msg_ptr);

    // Writing Point Cloud data to PCD file
    if (filter_res == 0.0)
    {
        pcl::io::savePCDFileASCII(filename, *map_ptr);
        std::cout << "Saved " << map_ptr->points.size() << " data points to " << filename << "." << std::endl;
    }
    else
    {
        pcl::io::savePCDFileASCII(filename, *map_filtered);
        std::cout << "Saved " << map_filtered->points.size() << " data points to " << filename << "." << std::endl;
    }
}

void ndt_mapping::imu_odom_calc(ros::Time current_time)
{
    ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();
    // std::cout << "imu_odom_calc diff_time = " << diff_time << std::endl;

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    ndt_mapping::current_pose_imu_odom.roll += diff_imu_roll;
    ndt_mapping::current_pose_imu_odom.pitch += diff_imu_pitch;
    ndt_mapping::current_pose_imu_odom.yaw += diff_imu_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    ndt_mapping::offset_imu_odom_x += diff_distance * cos(-ndt_mapping::current_pose_imu_odom.pitch) * cos(ndt_mapping::current_pose_imu_odom.yaw);
    ndt_mapping::offset_imu_odom_y += diff_distance * cos(-ndt_mapping::current_pose_imu_odom.pitch) * sin(ndt_mapping::current_pose_imu_odom.yaw);
    ndt_mapping::offset_imu_odom_z += diff_distance * sin(-ndt_mapping::current_pose_imu_odom.pitch);

    ndt_mapping::offset_imu_odom_roll += diff_imu_roll;
    ndt_mapping::offset_imu_odom_pitch += diff_imu_pitch;
    ndt_mapping::offset_imu_odom_yaw += diff_imu_yaw;

    ndt_mapping::guess_pose_imu_odom.x = ndt_mapping::previous_pose.x + ndt_mapping::offset_imu_odom_x;
    ndt_mapping::guess_pose_imu_odom.y = ndt_mapping::previous_pose.y + ndt_mapping::offset_imu_odom_y;
    ndt_mapping::guess_pose_imu_odom.z = ndt_mapping::previous_pose.z + ndt_mapping::offset_imu_odom_z;
    ndt_mapping::guess_pose_imu_odom.roll = ndt_mapping::previous_pose.roll + ndt_mapping::offset_imu_odom_roll;
    ndt_mapping::guess_pose_imu_odom.pitch = ndt_mapping::previous_pose.pitch + ndt_mapping::offset_imu_odom_pitch;
    ndt_mapping::guess_pose_imu_odom.yaw = ndt_mapping::previous_pose.yaw + ndt_mapping::offset_imu_odom_yaw;

    previous_time = current_time;
}

void ndt_mapping::odom_calc(ros::Time current_time)
{
    ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();
    // std::cout << "odom_calc diff_time = " << diff_time << std::endl;

    double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
    double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
    double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

    ndt_mapping::current_pose_odom.roll += diff_odom_roll;
    ndt_mapping::current_pose_odom.pitch += diff_odom_pitch;
    ndt_mapping::current_pose_odom.yaw += diff_odom_yaw;

    double diff_distance = odom.twist.twist.linear.x * diff_time;
    ndt_mapping::offset_odom_x += diff_distance * cos(-ndt_mapping::current_pose_odom.pitch) * cos(ndt_mapping::current_pose_odom.yaw);
    ndt_mapping::offset_odom_y += diff_distance * cos(-ndt_mapping::current_pose_odom.pitch) * sin(ndt_mapping::current_pose_odom.yaw);
    ndt_mapping::offset_odom_z += diff_distance * sin(-ndt_mapping::current_pose_odom.pitch);

    ndt_mapping::offset_odom_roll += diff_odom_roll;
    ndt_mapping::offset_odom_pitch += diff_odom_pitch;
    ndt_mapping::offset_odom_yaw += diff_odom_yaw;

    ndt_mapping::guess_pose_odom.x = ndt_mapping::previous_pose.x + ndt_mapping::offset_odom_x;
    ndt_mapping::guess_pose_odom.y = ndt_mapping::previous_pose.y + ndt_mapping::offset_odom_y;
    ndt_mapping::guess_pose_odom.z = ndt_mapping::previous_pose.z + ndt_mapping::offset_odom_z;
    ndt_mapping::guess_pose_odom.roll = ndt_mapping::previous_pose.roll + ndt_mapping::offset_odom_roll;
    ndt_mapping::guess_pose_odom.pitch = ndt_mapping::previous_pose.pitch + ndt_mapping::offset_odom_pitch;
    ndt_mapping::guess_pose_odom.yaw = ndt_mapping::previous_pose.yaw + ndt_mapping::offset_odom_yaw;

    previous_time = current_time;
}

void ndt_mapping::imu_calc(ros::Time current_time)
{
    ros::Time previous_time = current_time;
    double diff_time = (current_time - previous_time).toSec();
    // std::cout << "imu_calc diff_time = " << diff_time << std::endl;

    double diff_imu_roll = imu.angular_velocity.x * diff_time;
    double diff_imu_pitch = imu.angular_velocity.y * diff_time;
    double diff_imu_yaw = imu.angular_velocity.z * diff_time;

    ndt_mapping::current_pose_imu.roll += diff_imu_roll;
    ndt_mapping::current_pose_imu.pitch += diff_imu_pitch;
    ndt_mapping::current_pose_imu.yaw += diff_imu_yaw;

    double accX1 = imu.linear_acceleration.x;
    double accY1 = std::cos(ndt_mapping::current_pose_imu.roll) * imu.linear_acceleration.y -
                   std::sin(ndt_mapping::current_pose_imu.roll) * imu.linear_acceleration.z;
    double accZ1 = std::sin(ndt_mapping::current_pose_imu.roll) * imu.linear_acceleration.y +
                   std::cos(ndt_mapping::current_pose_imu.roll) * imu.linear_acceleration.z;

    double accX2 = std::sin(ndt_mapping::current_pose_imu.pitch) * accZ1 + std::cos(ndt_mapping::current_pose_imu.pitch) * accX1;
    double accY2 = accY1;
    double accZ2 = std::cos(ndt_mapping::current_pose_imu.pitch) * accZ1 - std::sin(ndt_mapping::current_pose_imu.pitch) * accX1;

    double accX = std::cos(ndt_mapping::current_pose_imu.yaw) * accX2 - std::sin(ndt_mapping::current_pose_imu.yaw) * accY2;
    double accY = std::sin(ndt_mapping::current_pose_imu.yaw) * accX2 + std::cos(ndt_mapping::current_pose_imu.yaw) * accY2;
    double accZ = accZ2;

    ndt_mapping::offset_imu_x += ndt_mapping::current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
    ndt_mapping::offset_imu_y += ndt_mapping::current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
    ndt_mapping::offset_imu_z += ndt_mapping::current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

    ndt_mapping::current_velocity_imu_x += accX * diff_time;
    ndt_mapping::current_velocity_imu_y += accY * diff_time;
    ndt_mapping::current_velocity_imu_z += accZ * diff_time;

    ndt_mapping::offset_imu_roll += diff_imu_roll;
    ndt_mapping::offset_imu_pitch += diff_imu_pitch;
    ndt_mapping::offset_imu_yaw += diff_imu_yaw;

    ndt_mapping::guess_pose_imu.x = ndt_mapping::previous_pose.x + ndt_mapping::offset_imu_x;
    ndt_mapping::guess_pose_imu.y = ndt_mapping::previous_pose.y + ndt_mapping::offset_imu_y;
    ndt_mapping::guess_pose_imu.z = ndt_mapping::previous_pose.z + ndt_mapping::offset_imu_z;
    ndt_mapping::guess_pose_imu.roll = ndt_mapping::previous_pose.roll + ndt_mapping::offset_imu_roll;
    ndt_mapping::guess_pose_imu.pitch = ndt_mapping::previous_pose.pitch + ndt_mapping::offset_imu_pitch;
    ndt_mapping::guess_pose_imu.yaw = ndt_mapping::previous_pose.yaw + ndt_mapping::offset_imu_yaw;

    previous_time = current_time;
}

double ndt_mapping::wrapToPm(double a_num, const double a_max)
{
    if (a_num >= a_max)
    {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

double ndt_mapping::wrapToPmPi(double a_angle_rad)
{
    return ndt_mapping::wrapToPm(a_angle_rad, M_PI);
}

double ndt_mapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

void ndt_mapping::odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
    // std::cout << __func__ << std::endl;

    odom = *input;
    // ndt_mapping::odom_calc(input->header.stamp);
}

void ndt_mapping::imuUpsideDown(const sensor_msgs::Imu::Ptr input)
{
    double input_roll, input_pitch, input_yaw;

    tf::Quaternion input_orientation;
    tf::quaternionMsgToTF(input->orientation, input_orientation);
    tf::Matrix3x3(input_orientation).getRPY(input_roll, input_pitch, input_yaw);

    input->angular_velocity.x *= -1;
    input->angular_velocity.y *= -1;
    input->angular_velocity.z *= -1;

    input->linear_acceleration.x *= -1;
    input->linear_acceleration.y *= -1;
    input->linear_acceleration.z *= -1;

    input_roll *= -1;
    input_pitch *= -1;
    input_yaw *= -1;

    input->orientation = tf::createQuaternionMsgFromRollPitchYaw(input_roll, input_pitch, input_yaw);
}

void ndt_mapping::imu_callback(const sensor_msgs::Imu::Ptr& input)
{
    // std::cout << __func__ << std::endl;

    if (ndt_mapping::_imu_upside_down)
        ndt_mapping::imuUpsideDown(input);

    const ros::Time current_time = input->header.stamp;
    ros::Time previous_time = current_time;
    const double diff_time = (current_time - previous_time).toSec();

    double imu_roll, imu_pitch, imu_yaw;
    tf::Quaternion imu_orientation;
    tf::quaternionMsgToTF(input->orientation, imu_orientation);
    tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

    imu_roll = ndt_mapping::wrapToPmPi(imu_roll);
    imu_pitch = ndt_mapping::wrapToPmPi(imu_pitch);
    imu_yaw = ndt_mapping::wrapToPmPi(imu_yaw);

    double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
    const double diff_imu_roll = ndt_mapping::calcDiffForRadian(imu_roll, previous_imu_roll);
    const double diff_imu_pitch = ndt_mapping::calcDiffForRadian(imu_pitch, previous_imu_pitch);
    const double diff_imu_yaw = ndt_mapping::calcDiffForRadian(imu_yaw, previous_imu_yaw);

    imu.header = input->header;
    imu.linear_acceleration.x = input->linear_acceleration.x;
    // imu.linear_acceleration.y = input->linear_acceleration.y;
    // imu.linear_acceleration.z = input->linear_acceleration.z;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;

    if (diff_time != 0)
    {
        imu.angular_velocity.x = diff_imu_roll / diff_time;
        imu.angular_velocity.y = diff_imu_pitch / diff_time;
        imu.angular_velocity.z = diff_imu_yaw / diff_time;
    }
    else
    {
        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = 0;
    }

    // ndt_mapping::imu_calc(input->header.stamp);

    previous_time = current_time;
    previous_imu_roll = imu_roll;
    previous_imu_pitch = imu_pitch;
    previous_imu_yaw = imu_yaw;
}

void ndt_mapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    double r;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q;

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ndt_mapping::current_scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, tmp);

    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (ndt_mapping::min_scan_range < r && r < ndt_mapping::max_scan_range)
        {
            scan.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    // Add initial point cloud to velodyne_map
    if (ndt_mapping::initial_scan_loaded == 0)
    {
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, ndt_mapping::tf_btol);
        map += *transformed_scan_ptr;
        ndt_mapping::initial_scan_loaded = 1;
    }

    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(ndt_mapping::voxel_leaf_size, ndt_mapping::voxel_leaf_size, ndt_mapping::voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

    if (ndt_mapping::_method_type == MethodType::PCL_GENERIC)
    {
        ndt.setTransformationEpsilon(ndt_mapping::trans_eps);
        ndt.setStepSize(ndt_mapping::step_size);
        ndt.setResolution(ndt_mapping::ndt_res);
        ndt.setMaximumIterations(ndt_mapping::max_iter);
        ndt.setInputSource(filtered_scan_ptr);
    }
    else if (ndt_mapping::_method_type == MethodType::PCL_ANH)
    {
        anh_ndt.setTransformationEpsilon(ndt_mapping::trans_eps);
        anh_ndt.setStepSize(ndt_mapping::step_size);
        anh_ndt.setResolution(ndt_mapping::ndt_res);
        anh_ndt.setMaximumIterations(ndt_mapping::max_iter);
        anh_ndt.setInputSource(filtered_scan_ptr);
    }

  #ifdef USE_PCL_OPENMP
    else if (ndt_mapping::_method_type == MethodType::PCL_OPENMP)
  {
    omp_ndt.setTransformationEpsilon(ndt_mapping::trans_eps);
    omp_ndt.setStepSize(ndt_mapping::step_size);
    omp_ndt.setResolution(ndt_mapping::ndt_res);
    omp_ndt.setMaximumIterations(ndt_mapping::max_iter);
    omp_ndt.setInputSource(filtered_scan_ptr);
  }
  #endif

    bool is_first_map = true;
    if (is_first_map == true)
    {
        if (ndt_mapping::_method_type == MethodType::PCL_GENERIC)
            ndt.setInputTarget(map_ptr);
        else if (ndt_mapping::_method_type == MethodType::PCL_ANH)
            anh_ndt.setInputTarget(map_ptr);

  #ifdef USE_PCL_OPENMP
        else if (ndt_mapping::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(map_ptr);
  #endif
        is_first_map = false;
    }

    ndt_mapping::guess_pose.x = ndt_mapping::previous_pose.x + ndt_mapping::diff_x;
    ndt_mapping::guess_pose.y = ndt_mapping::previous_pose.y + ndt_mapping::diff_y;
    ndt_mapping::guess_pose.z = ndt_mapping::previous_pose.z + ndt_mapping::diff_z;
    ndt_mapping::guess_pose.roll = ndt_mapping::previous_pose.roll;
    ndt_mapping::guess_pose.pitch = ndt_mapping::previous_pose.pitch;
    ndt_mapping::guess_pose.yaw = ndt_mapping::previous_pose.yaw + ndt_mapping::diff_yaw;

    // if (ndt_mapping::_use_imu == true && ndt_mapping::_use_odom == true)
    //     ndt_mapping::imu_odom_calc(ndt_mapping::current_scan_time);
    // if (ndt_mapping::_use_imu == true && ndt_mapping::_use_odom == false)
    //     ndt_mapping::imu_calc(ndt_mapping::current_scan_time);
    // if (ndt_mapping::_use_imu == false && ndt_mapping::_use_odom == true)
    //     ndt_mapping::odom_calc(ndt_mapping::current_scan_time);

    pose guess_pose_for_ndt;
    if (ndt_mapping::_use_imu == true && ndt_mapping::_use_odom == true)
        guess_pose_for_ndt = ndt_mapping::guess_pose_imu_odom;
    else if (ndt_mapping::_use_imu == true && ndt_mapping::_use_odom == false)
        guess_pose_for_ndt = ndt_mapping::guess_pose_imu;
    else if (ndt_mapping::_use_imu == false && ndt_mapping::_use_odom == true)
        guess_pose_for_ndt = ndt_mapping::guess_pose_odom;
    else
        guess_pose_for_ndt = ndt_mapping::guess_pose;

    Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

    Eigen::Matrix4f init_guess =
            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * ndt_mapping::tf_btol;

    ndt_mapping::t3_end = ros::Time::now();
    ndt_mapping::d3 = ndt_mapping::t3_end - ndt_mapping::t3_start;

    ndt_mapping::t4_start = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (ndt_mapping::_method_type == MethodType::PCL_GENERIC)
    {
        ndt.align(*output_cloud, init_guess);
        ndt_mapping::fitness_score = ndt.getFitnessScore();
        t_localizer = ndt.getFinalTransformation();
        ndt_mapping::has_converged = ndt.hasConverged();
        ndt_mapping::final_num_iteration = ndt.getFinalNumIteration();
        ndt_mapping::transformation_probability = ndt.getTransformationProbability();
    }
    else if (ndt_mapping::_method_type == MethodType::PCL_ANH)
    {
        anh_ndt.align(init_guess);
        ndt_mapping::fitness_score = anh_ndt.getFitnessScore();
        t_localizer = anh_ndt.getFinalTransformation();
        ndt_mapping::has_converged = anh_ndt.hasConverged();
        ndt_mapping::final_num_iteration = anh_ndt.getFinalNumIteration();
    }

#ifdef USE_PCL_OPENMP
    else if (ndt_mapping::_method_type == MethodType::PCL_OPENMP)
  {
    omp_ndt.align(*output_cloud, init_guess);
    ndt_mapping::fitness_score = omp_ndt.getFitnessScore();
    t_localizer = omp_ndt.getFinalTransformation();
    ndt_mapping::has_converged = omp_ndt.hasConverged();
    ndt_mapping::final_num_iteration = omp_ndt.getFinalNumIteration();
  }
#endif

    t_base_link = t_localizer * ndt_mapping::tf_ltob;

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_l, mat_b;

    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));

    // Update localizer_pose.
    ndt_mapping::localizer_pose.x = t_localizer(0, 3);
    ndt_mapping::localizer_pose.y = t_localizer(1, 3);
    ndt_mapping::localizer_pose.z = t_localizer(2, 3);
    mat_l.getRPY(ndt_mapping::localizer_pose.roll, ndt_mapping::localizer_pose.pitch, ndt_mapping::localizer_pose.yaw, 1);

    // Update ndt_pose.
    ndt_mapping::ndt_pose.x = t_base_link(0, 3);
    ndt_mapping::ndt_pose.y = t_base_link(1, 3);
    ndt_mapping::ndt_pose.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_mapping::ndt_pose.roll, ndt_mapping::ndt_pose.pitch, ndt_mapping::ndt_pose.yaw, 1);

    ndt_mapping::current_pose.x = ndt_mapping::ndt_pose.x;
    ndt_mapping::current_pose.y = ndt_mapping::ndt_pose.y;
    ndt_mapping::current_pose.z = ndt_mapping::ndt_pose.z;
    ndt_mapping::current_pose.roll = ndt_mapping::ndt_pose.roll;
    ndt_mapping::current_pose.pitch = ndt_mapping::ndt_pose.pitch;
    ndt_mapping::current_pose.yaw = ndt_mapping::ndt_pose.yaw;

    transform.setOrigin(tf::Vector3(ndt_mapping::current_pose.x, ndt_mapping::current_pose.y, ndt_mapping::current_pose.z));
    q.setRPY(ndt_mapping::current_pose.roll, ndt_mapping::current_pose.pitch, ndt_mapping::current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ndt_mapping::current_scan_time, "map", "base_link"));

    ndt_mapping::scan_duration = ndt_mapping::current_scan_time - ndt_mapping::previous_scan_time;
    double secs = ndt_mapping::scan_duration.toSec();

    // Calculate the offset (curren_pos - previous_pos)
    ndt_mapping::diff_x = ndt_mapping::current_pose.x - ndt_mapping::previous_pose.x;
    ndt_mapping::diff_y = ndt_mapping::current_pose.y - ndt_mapping::previous_pose.y;
    ndt_mapping::diff_z = ndt_mapping::current_pose.z - ndt_mapping::previous_pose.z;
    ndt_mapping::diff_yaw = ndt_mapping::calcDiffForRadian(ndt_mapping::current_pose.yaw, ndt_mapping::previous_pose.yaw);
    ndt_mapping::diff = sqrt(ndt_mapping::diff_x * ndt_mapping::diff_x + ndt_mapping::diff_y * ndt_mapping::diff_y + ndt_mapping::diff_z * ndt_mapping::diff_z);

    ndt_mapping::current_velocity_x = ndt_mapping::diff_x / secs;
    ndt_mapping::current_velocity_y = ndt_mapping::diff_y / secs;
    ndt_mapping::current_velocity_z = ndt_mapping::diff_z / secs;

    ndt_mapping::current_pose_imu.x = ndt_mapping::current_pose.x;
    ndt_mapping::current_pose_imu.y = ndt_mapping::current_pose.y;
    ndt_mapping::current_pose_imu.z = ndt_mapping::current_pose.z;
    ndt_mapping::current_pose_imu.roll = ndt_mapping::current_pose.roll;
    ndt_mapping::current_pose_imu.pitch = ndt_mapping::current_pose.pitch;
    ndt_mapping::current_pose_imu.yaw = ndt_mapping::current_pose.yaw;

    ndt_mapping::current_pose_odom.x = ndt_mapping::current_pose.x;
    ndt_mapping::current_pose_odom.y = ndt_mapping::current_pose.y;
    ndt_mapping::current_pose_odom.z = ndt_mapping::current_pose.z;
    ndt_mapping::current_pose_odom.roll = ndt_mapping::current_pose.roll;
    ndt_mapping::current_pose_odom.pitch = ndt_mapping::current_pose.pitch;
    ndt_mapping::current_pose_odom.yaw = ndt_mapping::current_pose.yaw;

    ndt_mapping::current_pose_imu_odom.x = ndt_mapping::current_pose.x;
    ndt_mapping::current_pose_imu_odom.y = ndt_mapping::current_pose.y;
    ndt_mapping::current_pose_imu_odom.z = ndt_mapping::current_pose.z;
    ndt_mapping::current_pose_imu_odom.roll = ndt_mapping::current_pose.roll;
    ndt_mapping::current_pose_imu_odom.pitch = ndt_mapping::current_pose.pitch;
    ndt_mapping::current_pose_imu_odom.yaw = ndt_mapping::current_pose.yaw;

    ndt_mapping::current_velocity_imu_x = ndt_mapping::current_velocity_x;
    ndt_mapping::current_velocity_imu_y = ndt_mapping::current_velocity_y;
    ndt_mapping::current_velocity_imu_z = ndt_mapping::current_velocity_z;

    // Update position and posture. current_pos -> previous_pos
    ndt_mapping::previous_pose.x = ndt_mapping::current_pose.x;
    ndt_mapping::previous_pose.y = ndt_mapping::current_pose.y;
    ndt_mapping::previous_pose.z = ndt_mapping::current_pose.z;
    ndt_mapping::previous_pose.roll = ndt_mapping::current_pose.roll;
    ndt_mapping::previous_pose.pitch = ndt_mapping::current_pose.pitch;
    ndt_mapping::previous_pose.yaw = ndt_mapping::current_pose.yaw;

    ndt_mapping::previous_scan_time.sec = ndt_mapping::current_scan_time.sec;
    ndt_mapping::previous_scan_time.nsec = ndt_mapping::current_scan_time.nsec;

    ndt_mapping::offset_imu_x = 0.0;
    ndt_mapping::offset_imu_y = 0.0;
    ndt_mapping::offset_imu_z = 0.0;
    ndt_mapping::offset_imu_roll = 0.0;
    ndt_mapping::offset_imu_pitch = 0.0;
    ndt_mapping::offset_imu_yaw = 0.0;

    ndt_mapping::offset_odom_x = 0.0;
    ndt_mapping::offset_odom_y = 0.0;
    ndt_mapping::offset_odom_z = 0.0;
    ndt_mapping::offset_odom_roll = 0.0;
    ndt_mapping::offset_odom_pitch = 0.0;
    ndt_mapping::offset_odom_yaw = 0.0;

    ndt_mapping::offset_imu_odom_x = 0.0;
    ndt_mapping::offset_imu_odom_y = 0.0;
    ndt_mapping::offset_imu_odom_z = 0.0;
    ndt_mapping::offset_imu_odom_roll = 0.0;
    ndt_mapping::offset_imu_odom_pitch = 0.0;
    ndt_mapping::offset_imu_odom_yaw = 0.0;

    // Calculate the shift between added_pos and current_pos
    double shift = sqrt(pow(ndt_mapping::current_pose.x - ndt_mapping::added_pose.x, 2.0) + pow(ndt_mapping::current_pose.y - ndt_mapping::added_pose.y, 2.0));
    if (shift >= min_add_scan_shift)
    {
        map += *transformed_scan_ptr;
        ndt_mapping::added_pose.x = ndt_mapping::current_pose.x;
        ndt_mapping::added_pose.y = ndt_mapping::current_pose.y;
        ndt_mapping::added_pose.z = ndt_mapping::current_pose.z;
        ndt_mapping::added_pose.roll = ndt_mapping::current_pose.roll;
        ndt_mapping::added_pose.pitch = ndt_mapping::current_pose.pitch;
        ndt_mapping::added_pose.yaw = ndt_mapping::current_pose.yaw;

        if (ndt_mapping::_method_type == MethodType::PCL_GENERIC)
            ndt.setInputTarget(map_ptr);
        else if (ndt_mapping::_method_type == MethodType::PCL_ANH)
        {
            if (ndt_mapping::_incremental_voxel_update == true)
                anh_ndt.updateVoxelGrid(transformed_scan_ptr);
            else
                anh_ndt.setInputTarget(map_ptr);
        }

#ifdef USE_PCL_OPENMP
        else if (ndt_mapping::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputTarget(map_ptr);
#endif
    }

    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    ndt_mapping::ndt_map_pub.publish(*map_msg_ptr);

    q.setRPY(ndt_mapping::current_pose.roll, ndt_mapping::current_pose.pitch, ndt_mapping::current_pose.yaw);
    ndt_mapping::current_pose_msg.header.frame_id = "map";
    ndt_mapping::current_pose_msg.header.stamp = ndt_mapping::current_scan_time;
    ndt_mapping::current_pose_msg.pose.position.x = ndt_mapping::current_pose.x;
    ndt_mapping::current_pose_msg.pose.position.y = ndt_mapping::current_pose.y;
    ndt_mapping::current_pose_msg.pose.position.z = ndt_mapping::current_pose.z;
    ndt_mapping::current_pose_msg.pose.orientation.x = q.x();
    ndt_mapping::current_pose_msg.pose.orientation.y = q.y();
    ndt_mapping::current_pose_msg.pose.orientation.z = q.z();
    ndt_mapping::current_pose_msg.pose.orientation.w = q.w();

    ndt_mapping::current_pose_pub.publish(ndt_mapping::current_pose_msg);

    // Write log
    if (!ndt_mapping::ofs)
    {
        std::cerr << "Could not open " << ndt_mapping::filename << "." << std::endl;
        exit(1);
    }

    ndt_mapping::ofs << input->header.seq << ","
        << input->header.stamp << ","
        << input->header.frame_id << ","
        << scan_ptr->size() << ","
        << filtered_scan_ptr->size() << ","
        << std::fixed << std::setprecision(5) << ndt_mapping::current_pose.x << ","
        << std::fixed << std::setprecision(5) << ndt_mapping::current_pose.y << ","
        << std::fixed << std::setprecision(5) << ndt_mapping::current_pose.z << ","
        << ndt_mapping::current_pose.roll << ","
        << ndt_mapping::current_pose.pitch << ","
        << ndt_mapping::current_pose.yaw << ","
        << ndt_mapping::final_num_iteration << ","
        << ndt_mapping::fitness_score << ","
        << ndt_mapping::ndt_res << ","
        << ndt_mapping::step_size << ","
        << ndt_mapping::trans_eps << ","
        << ndt_mapping::max_iter << ","
        << ndt_mapping::voxel_leaf_size << ","
        << ndt_mapping::min_scan_range << ","
        << ndt_mapping::max_scan_range << ","
        << min_add_scan_shift << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
    std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << ndt_mapping::has_converged << std::endl;
    std::cout << "Fitness score: " << ndt_mapping::fitness_score << std::endl;
    std::cout << "Number of iteration: " << ndt_mapping::final_num_iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << ndt_mapping::current_pose.x << ", " << ndt_mapping::current_pose.y << ", " << ndt_mapping::current_pose.z << ", " << ndt_mapping::current_pose.roll
              << ", " << ndt_mapping::current_pose.pitch << ", " << ndt_mapping::current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
}

int ndt_mapping::RUN(int argc, char** argv)
{
    ndt_mapping::previous_pose.x = 0.0;
    ndt_mapping::previous_pose.y = 0.0;
    ndt_mapping::previous_pose.z = 0.0;
    ndt_mapping::previous_pose.roll = 0.0;
    ndt_mapping::previous_pose.pitch = 0.0;
    ndt_mapping::previous_pose.yaw = 0.0;

    ndt_mapping::ndt_pose.x = 0.0;
    ndt_mapping::ndt_pose.y = 0.0;
    ndt_mapping::ndt_pose.z = 0.0;
    ndt_mapping::ndt_pose.roll = 0.0;
    ndt_mapping::ndt_pose.pitch = 0.0;
    ndt_mapping::ndt_pose.yaw = 0.0;

    ndt_mapping::current_pose.x = 0.0;
    ndt_mapping::current_pose.y = 0.0;
    ndt_mapping::current_pose.z = 0.0;
    ndt_mapping::current_pose.roll = 0.0;
    ndt_mapping::current_pose.pitch = 0.0;
    ndt_mapping::current_pose.yaw = 0.0;

    ndt_mapping::current_pose_imu.x = 0.0;
    ndt_mapping::current_pose_imu.y = 0.0;
    ndt_mapping::current_pose_imu.z = 0.0;
    ndt_mapping::current_pose_imu.roll = 0.0;
    ndt_mapping::current_pose_imu.pitch = 0.0;
    ndt_mapping::current_pose_imu.yaw = 0.0;

    ndt_mapping::guess_pose.x = 0.0;
    ndt_mapping::guess_pose.y = 0.0;
    ndt_mapping::guess_pose.z = 0.0;
    ndt_mapping::guess_pose.roll = 0.0;
    ndt_mapping::guess_pose.pitch = 0.0;
    ndt_mapping::guess_pose.yaw = 0.0;

    ndt_mapping::added_pose.x = 0.0;
    ndt_mapping::added_pose.y = 0.0;
    ndt_mapping::added_pose.z = 0.0;
    ndt_mapping::added_pose.roll = 0.0;
    ndt_mapping::added_pose.pitch = 0.0;
    ndt_mapping::added_pose.yaw = 0.0;

    ndt_mapping::diff_x = 0.0;
    ndt_mapping::diff_y = 0.0;
    ndt_mapping::diff_z = 0.0;
    ndt_mapping::diff_yaw = 0.0;

    ndt_mapping::offset_imu_x = 0.0;
    ndt_mapping::offset_imu_y = 0.0;
    ndt_mapping::offset_imu_z = 0.0;
    ndt_mapping::offset_imu_roll = 0.0;
    ndt_mapping::offset_imu_pitch = 0.0;
    ndt_mapping::offset_imu_yaw = 0.0;

    ndt_mapping::offset_odom_x = 0.0;
    ndt_mapping::offset_odom_y = 0.0;
    ndt_mapping::offset_odom_z = 0.0;
    ndt_mapping::offset_odom_roll = 0.0;
    ndt_mapping::offset_odom_pitch = 0.0;
    ndt_mapping::offset_odom_yaw = 0.0;

    ndt_mapping::offset_imu_odom_x = 0.0;
    ndt_mapping::offset_imu_odom_y = 0.0;
    ndt_mapping::offset_imu_odom_z = 0.0;
    ndt_mapping::offset_imu_odom_roll = 0.0;
    ndt_mapping::offset_imu_odom_pitch = 0.0;
    ndt_mapping::offset_imu_odom_yaw = 0.0;

    ros::init(argc, argv, "ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set log file name.
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm* pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    ndt_mapping::filename = "ndt_mapping_" + std::string(buffer) + ".csv";
    ndt_mapping::ofs.open(ndt_mapping::filename.c_str(), std::ios::app);

    // write header for log file
    if (!ndt_mapping::ofs)
    {
        std::cerr << "Could not open " << ndt_mapping::filename << "." << std::endl;
        exit(1);
    }

    ndt_mapping::ofs << "input->header.seq" << ","
        << "input->header.stamp" << ","
        << "input->header.frame_id" << ","
        << "scan_ptr->size()" << ","
        << "filtered_scan_ptr->size()" << ","
        << "current_pose.x" << ","
        << "current_pose.y" << ","
        << "current_pose.z" << ","
        << "current_pose.roll" << ","
        << "current_pose.pitch" << ","
        << "current_pose.yaw" << ","
        << "final_num_iteration" << ","
        << "fitness_score" << ","
        << "ndt_res" << ","
        << "step_size" << ","
        << "trans_eps" << ","
        << "max_iter" << ","
        << "voxel_leaf_size" << ","
        << "min_scan_range" << ","
        << "max_scan_range" << ","
        << "min_add_scan_shift" << std::endl;

    // setting parameters
    int method_type_tmp = 0;
    private_nh.getParam("method_type", method_type_tmp);
    ndt_mapping::_method_type = static_cast<MethodType>(method_type_tmp);
    private_nh.getParam("imu_upside_down", ndt_mapping::_imu_upside_down);
    private_nh.getParam("imu_topic", ndt_mapping::_imu_topic);
    private_nh.getParam("incremental_voxel_update", ndt_mapping::_incremental_voxel_update);
    private_nh.getParam("use_imu", ndt_mapping::_use_imu);
    private_nh.getParam("use_odom", ndt_mapping::_use_odom);

    std::cout << "method_type: " << static_cast<int>(ndt_mapping::_method_type) << std::endl;
    std::cout << "use_odom: " << ndt_mapping::_use_odom << std::endl;
    std::cout << "use_imu: " << ndt_mapping::_use_imu << std::endl;
    std::cout << "imu_upside_down: " << ndt_mapping::_imu_upside_down << std::endl;
    std::cout << "imu_topic: " << ndt_mapping::_imu_topic << std::endl;
    std::cout << "incremental_voxel_update: " << ndt_mapping::_incremental_voxel_update << std::endl;

    if (nh.getParam("tf_x", ndt_mapping::_tf_x) == false)
    {
        std::cout << "tf_x is not set." << std::endl;
        return 1;
    }
    if (nh.getParam("tf_y", ndt_mapping::_tf_y) == false)
    {
        std::cout << "tf_y is not set." << std::endl;
        return 1;
    }
    if (nh.getParam("tf_z", ndt_mapping::_tf_z) == false)
    {
        std::cout << "tf_z is not set." << std::endl;
        return 1;
    }
    if (nh.getParam("tf_roll", ndt_mapping::_tf_roll) == false)
    {
        std::cout << "tf_roll is not set." << std::endl;
        return 1;
    }
    if (nh.getParam("tf_pitch", ndt_mapping::_tf_pitch) == false)
    {
        std::cout << "tf_pitch is not set." << std::endl;
        return 1;
    }
    if (nh.getParam("tf_yaw", ndt_mapping::_tf_yaw) == false)
    {
        std::cout << "tf_yaw is not set." << std::endl;
        return 1;
    }

    std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << ndt_mapping::_tf_x << ", " << ndt_mapping::_tf_y << ", " << ndt_mapping::_tf_z << ", "
              << ndt_mapping::_tf_roll << ", " << ndt_mapping::_tf_pitch << ", " << ndt_mapping::_tf_yaw << ")" << std::endl;


#ifndef USE_PCL_OPENMP
    if (ndt_mapping::_method_type == MethodType::PCL_OPENMP)
    {
        std::cerr << "**************************************************************" << std::endl;
        std::cerr << "[ERROR]PCL_OPENMP is not built. Please use other method type." << std::endl;
        std::cerr << "**************************************************************" << std::endl;
        exit(1);
    }
#endif

    Eigen::Translation3f tl_btol(ndt_mapping::_tf_x, _tf_y, _tf_z);                 // tl: translation
    Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
    ndt_mapping::tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    ndt_mapping::tf_ltob = ndt_mapping::tf_btol.inverse();

    map.header.frame_id = "map";

    ndt_mapping::ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    ndt_mapping::current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    ros::Subscriber param_sub = nh.subscribe("config/ndt_mapping", 10, &ndt_mapping::param_callback, this);
    ros::Subscriber output_sub = nh.subscribe("config/ndt_mapping_output", 10, &ndt_mapping::output_callback, this);
    ros::Subscriber points_sub = nh.subscribe("points_raw", 100000, &ndt_mapping::points_callback, this);
    // ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", 100000, &ndt_mapping::odom_callback, this);
    ros::Subscriber imu_sub = nh.subscribe(ndt_mapping::_imu_topic, 100000, &ndt_mapping::imu_callback, this);

    ros::spin();

    return 0;
}
