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
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <std_msgs/String.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>

#ifdef USE_PCL_OPENMP
#include <pcl_omp_registration/ndt.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <aslan_msgs/ConfigNDT.h>

#include <aslan_msgs/NDTStat.h>

#include "ndt_matching.h"

void ndt_matching::param_callback(const aslan_msgs::ConfigNDT::ConstPtr& input)
{
    if (ndt_matching::_use_gnss != input->init_pos_gnss)
    {
        ndt_matching::init_pos_set = 0;
    }
    else if (ndt_matching::_use_gnss == 0 &&
            (ndt_matching::initial_pose.x != input->x || ndt_matching::initial_pose.y != input->y || ndt_matching::initial_pose.z != input->z ||
             ndt_matching::initial_pose.roll != input->roll || ndt_matching::initial_pose.pitch != input->pitch || ndt_matching::initial_pose.yaw != input->yaw))
    {
        ndt_matching::init_pos_set = 0;
    }

    ndt_matching::_use_gnss = input->init_pos_gnss;

    // Setting parameters
    if (input->resolution != ndt_matching::ndt_res)
    {
        ndt_matching::ndt_res = input->resolution;

        if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
            ndt.setResolution(ndt_matching::ndt_res);
        else if (ndt_matching::_method_type == MethodType::PCL_ANH)
            anh_ndt.setResolution(ndt_matching::ndt_res);
  #ifdef USE_PCL_OPENMP
    else if (ndt_matching::_method_type == MethodType::PCL_OPENMP) omp_ndt.setResolution(ndt_matching::ndt_res);
  #endif
    }

    if (input->step_size != ndt_matching::step_size)
    {
        ndt_matching::step_size = input->step_size;

        if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
            ndt.setStepSize(ndt_matching::step_size);
        else if (ndt_matching::_method_type == MethodType::PCL_ANH)
            anh_ndt.setStepSize(ndt_matching::step_size);
  #ifdef USE_PCL_OPENMP
        else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setStepSize(ndt_matching::ndt_res);
  #endif
    }

    if (input->trans_epsilon != ndt_matching::trans_eps)
    {
        ndt_matching::trans_eps = input->trans_epsilon;

        if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
            ndt.setTransformationEpsilon(ndt_matching::trans_eps);
        else if (ndt_matching::_method_type == MethodType::PCL_ANH)
            anh_ndt.setTransformationEpsilon(ndt_matching::trans_eps);
  #ifdef USE_PCL_OPENMP
        else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setTransformationEpsilon(ndt_matching::ndt_res);
  #endif
    }

    if (input->max_iterations != ndt_matching::max_iter)
    {
        ndt_matching::max_iter = input->max_iterations;

        if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
            ndt.setMaximumIterations(ndt_matching::max_iter);
        else if (ndt_matching::_method_type == MethodType::PCL_ANH)
            anh_ndt.setMaximumIterations(ndt_matching::max_iter);
  #ifdef USE_PCL_OPENMP
        else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setMaximumIterations(ndt_matching::ndt_res);
  #endif
    }

    if (ndt_matching::_use_gnss == 0 && ndt_matching::init_pos_set == 0)
    {
        ndt_matching::initial_pose.x = input->x;
        ndt_matching::initial_pose.y = input->y;
        ndt_matching::initial_pose.z = input->z;
        ndt_matching::initial_pose.roll = input->roll;
        ndt_matching::initial_pose.pitch = input->pitch;
        ndt_matching::initial_pose.yaw = input->yaw;

        if (ndt_matching::_use_local_transform == true)
        {
            tf::Vector3 v(input->x, input->y, input->z);
            tf::Quaternion q;
            q.setRPY(input->roll, input->pitch, input->yaw);
            tf::Transform transform(q, v);
            ndt_matching::initial_pose.x = (local_transform.inverse() * transform).getOrigin().getX();
            ndt_matching::initial_pose.y = (local_transform.inverse() * transform).getOrigin().getY();
            ndt_matching::initial_pose.z = (local_transform.inverse() * transform).getOrigin().getZ();

            tf::Matrix3x3 m(q);
            m.getRPY(ndt_matching::initial_pose.roll, ndt_matching::initial_pose.pitch, ndt_matching::initial_pose.yaw);

            std::cout << "ndt_matching::initial_pose.x: " << ndt_matching::initial_pose.x << std::endl;
            std::cout << "ndt_matching::initial_pose.y: " << ndt_matching::initial_pose.y << std::endl;
            std::cout << "ndt_matching::initial_pose.z: " << ndt_matching::initial_pose.z << std::endl;
            std::cout << "ndt_matching::initial_pose.roll: " << ndt_matching::initial_pose.roll << std::endl;
            std::cout << "ndt_matching::initial_pose.pitch: " << ndt_matching::initial_pose.pitch << std::endl;
            std::cout << "ndt_matching::initial_pose.yaw: " << ndt_matching::initial_pose.yaw << std::endl;
        }

        // Setting position and posture for the first time.
        ndt_matching::localizer_pose.x = ndt_matching::initial_pose.x;
        ndt_matching::localizer_pose.y = ndt_matching::initial_pose.y;
        ndt_matching::localizer_pose.z = ndt_matching::initial_pose.z;
        ndt_matching::localizer_pose.roll = ndt_matching::initial_pose.roll;
        ndt_matching::localizer_pose.pitch = ndt_matching::initial_pose.pitch;
        ndt_matching::localizer_pose.yaw = ndt_matching::initial_pose.yaw;

        ndt_matching::previous_pose.x = ndt_matching::initial_pose.x;
        ndt_matching::previous_pose.y = ndt_matching::initial_pose.y;
        ndt_matching::previous_pose.z = ndt_matching::initial_pose.z;
        ndt_matching::previous_pose.roll = ndt_matching::initial_pose.roll;
        ndt_matching::previous_pose.pitch = ndt_matching::initial_pose.pitch;
        ndt_matching::previous_pose.yaw = ndt_matching::initial_pose.yaw;

        ndt_matching::current_pose.x = ndt_matching::initial_pose.x;
        ndt_matching::current_pose.y = ndt_matching::initial_pose.y;
        ndt_matching::current_pose.z = ndt_matching::initial_pose.z;
        ndt_matching::current_pose.roll = ndt_matching::initial_pose.roll;
        ndt_matching::current_pose.pitch = ndt_matching::initial_pose.pitch;
        ndt_matching::current_pose.yaw = ndt_matching::initial_pose.yaw;

        ndt_matching::current_velocity = 0;
        ndt_matching::current_velocity_x = 0;
        ndt_matching::current_velocity_y = 0;
        ndt_matching::current_velocity_z = 0;
        ndt_matching::angular_velocity = 0;

        ndt_matching::current_pose_imu.x = 0;
        ndt_matching::current_pose_imu.y = 0;
        ndt_matching::current_pose_imu.z = 0;
        ndt_matching::current_pose_imu.roll = 0;
        ndt_matching::current_pose_imu.pitch = 0;
        ndt_matching::current_pose_imu.yaw = 0;

        ndt_matching::current_velocity_imu_x = ndt_matching::current_velocity_x;
        ndt_matching::current_velocity_imu_y = ndt_matching::current_velocity_y;
        ndt_matching::current_velocity_imu_z = ndt_matching::current_velocity_z;
        ndt_matching::init_pos_set = 1;
    }
}

void ndt_matching::map_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  // if (ndt_matching::map_loaded == 0)
  if (ndt_matching::points_map_num != input->width)
  {
    std::cout << "Update points_map." << std::endl;

    ndt_matching::points_map_num = input->width;

    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map);

    if (ndt_matching::_use_local_transform == true)
    {
      tf::TransformListener local_transform_listener;
      try
      {
        ros::Time now = ros::Time(0);
        local_transform_listener.waitForTransform("/map", "/world", now, ros::Duration(10.0));
        local_transform_listener.lookupTransform("/map", "world", now, local_transform);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      pcl_ros::transformPointCloud(map, map, local_transform.inverse());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    // Setting point cloud to be aligned to.
    if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
    {
      pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_ndt.setResolution(ndt_matching::ndt_res);
      new_ndt.setInputTarget(map_ptr);
      new_ndt.setMaximumIterations(ndt_matching::max_iter);
      new_ndt.setStepSize(ndt_matching::step_size);
      new_ndt.setTransformationEpsilon(ndt_matching::trans_eps);

      new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&(ndt_matching::mutex));
      ndt = new_ndt;
      pthread_mutex_unlock(&(ndt_matching::mutex));
    }
    else if (ndt_matching::_method_type == MethodType::PCL_ANH)
    {
      cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
      new_anh_ndt.setResolution(ndt_matching::ndt_res);
      new_anh_ndt.setInputTarget(map_ptr);
      new_anh_ndt.setMaximumIterations(ndt_matching::max_iter);
      new_anh_ndt.setStepSize(ndt_matching::step_size);
      new_anh_ndt.setTransformationEpsilon(ndt_matching::trans_eps);

      pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointXYZ dummy_point;
      dummy_scan_ptr->push_back(dummy_point);
      new_anh_ndt.setInputSource(dummy_scan_ptr);

      new_anh_ndt.align(Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&(ndt_matching::mutex));
      anh_ndt = new_anh_ndt;
      pthread_mutex_unlock(&(ndt_matching::mutex));
    }

  #ifdef USE_PCL_OPENMP
    else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
    {
      pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_omp_ndt;
      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      new_omp_ndt.setResolution(ndt_matching::ndt_res);
      new_omp_ndt.setInputTarget(map_ptr);
      new_omp_ndt.setMaximumIterations(ndt_matching::max_iter);
      new_omp_ndt.setStepSize(ndt_matching::step_size);
      new_omp_ndt.setTransformationEpsilon(ndt_matching::trans_eps);

      new_omp_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

      pthread_mutex_lock(&(ndt_matching::mutex));
      omp_ndt = new_omp_ndt;
      pthread_mutex_unlock(&(ndt_matching::mutex));
    }
  #endif
    ndt_matching::map_loaded = 1;
  }
}

void ndt_matching::gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                        input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);

  pose current_gnss_pose;
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

   pose previous_gnss_pose = current_gnss_pose;
  ros::Time current_gnss_time = input->header.stamp;
   ros::Time previous_gnss_time = current_gnss_time;

  if ((ndt_matching::_use_gnss == 1 && ndt_matching::init_pos_set == 0) || ndt_matching::fitness_score >= 500.0)
  {
    ndt_matching::previous_pose.x = previous_gnss_pose.x;
    ndt_matching::previous_pose.y = previous_gnss_pose.y;
    ndt_matching::previous_pose.z = previous_gnss_pose.z;
    ndt_matching::previous_pose.roll = previous_gnss_pose.roll;
    ndt_matching::previous_pose.pitch = previous_gnss_pose.pitch;
    ndt_matching::previous_pose.yaw = previous_gnss_pose.yaw;

    ndt_matching::current_pose.x = current_gnss_pose.x;
    ndt_matching::current_pose.y = current_gnss_pose.y;
    ndt_matching::current_pose.z = current_gnss_pose.z;
    ndt_matching::current_pose.roll = current_gnss_pose.roll;
    ndt_matching::current_pose.pitch = current_gnss_pose.pitch;
    ndt_matching::current_pose.yaw = current_gnss_pose.yaw;

    ndt_matching::current_pose_imu = ndt_matching::current_pose_odom = ndt_matching::current_pose_imu_odom = ndt_matching::current_pose;

    ndt_matching::diff_x = ndt_matching::current_pose.x - ndt_matching::previous_pose.x;
    ndt_matching::diff_y = ndt_matching::current_pose.y - ndt_matching::previous_pose.y;
    ndt_matching::diff_z = ndt_matching::current_pose.z - ndt_matching::previous_pose.z;
    ndt_matching::diff_yaw = ndt_matching::current_pose.yaw - ndt_matching::previous_pose.yaw;
    ndt_matching::diff = sqrt(ndt_matching::diff_x * ndt_matching::diff_x + ndt_matching::diff_y * ndt_matching::diff_y + ndt_matching::diff_z * ndt_matching::diff_z);

    const double diff_time = (current_gnss_time - previous_gnss_time).toSec();
    ndt_matching::current_velocity = (diff_time > 0) ? (ndt_matching::diff / diff_time) : 0;
    ndt_matching::current_velocity_x = (diff_time > 0) ? (ndt_matching::diff_x / diff_time) : 0;
    ndt_matching::current_velocity_y = (diff_time > 0) ? (ndt_matching::diff_y / diff_time) : 0;
    ndt_matching::current_velocity_z = (diff_time > 0) ? (ndt_matching::diff_z / diff_time) : 0;
    ndt_matching::angular_velocity = (diff_time > 0) ? (ndt_matching::diff_yaw / diff_time) : 0;

    ndt_matching::current_accel = 0.0;
    ndt_matching::current_accel_x = 0.0;
    ndt_matching::current_accel_y = 0.0;
    ndt_matching::current_accel_z = 0.0;

    ndt_matching::init_pos_set = 1;
  }

  previous_gnss_pose.x = current_gnss_pose.x;
  previous_gnss_pose.y = current_gnss_pose.y;
  previous_gnss_pose.z = current_gnss_pose.z;
  previous_gnss_pose.roll = current_gnss_pose.roll;
  previous_gnss_pose.pitch = current_gnss_pose.pitch;
  previous_gnss_pose.yaw = current_gnss_pose.yaw;
  previous_gnss_time = current_gnss_time;
}

void ndt_matching::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    ros::Time now = ros::Time(0);
    listener.waitForTransform("/map", input->header.frame_id, now, ros::Duration(10.0));
    listener.lookupTransform("/map", input->header.frame_id, now, transform);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z,
                   input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);

  if (ndt_matching::_use_local_transform == true)
  {
    ndt_matching::current_pose.x = input->pose.pose.position.x;
    ndt_matching::current_pose.y = input->pose.pose.position.y;
    ndt_matching::current_pose.z = input->pose.pose.position.z;
  }
  else
  {
    ndt_matching::current_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
    ndt_matching::current_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
    ndt_matching::current_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  }
  m.getRPY(ndt_matching::current_pose.roll, ndt_matching::current_pose.pitch, ndt_matching::current_pose.yaw);

  if (ndt_matching::_get_height == true && ndt_matching::map_loaded == 1)
  {
    double min_distance = DBL_MAX;
    double nearest_z = ndt_matching::current_pose.z;
    for (const auto& p : map)
    {
      double distance = hypot(ndt_matching::current_pose.x - p.x, ndt_matching::current_pose.y - p.y);
      if (distance < min_distance)
      {
        min_distance = distance;
        nearest_z = p.z;
      }
    }
    ndt_matching::current_pose.z = nearest_z;
  }

  ndt_matching::current_pose_imu = ndt_matching::current_pose_odom = ndt_matching::current_pose_imu_odom = ndt_matching::current_pose;
  ndt_matching::previous_pose.x = ndt_matching::current_pose.x;
  ndt_matching::previous_pose.y = ndt_matching::current_pose.y;
  ndt_matching::previous_pose.z = ndt_matching::current_pose.z;
  ndt_matching::previous_pose.roll = ndt_matching::current_pose.roll;
  ndt_matching::previous_pose.pitch = ndt_matching::current_pose.pitch;
  ndt_matching::previous_pose.yaw = ndt_matching::current_pose.yaw;

  ndt_matching::current_velocity = 0.0;
  ndt_matching::current_velocity_x = 0.0;
  ndt_matching::current_velocity_y = 0.0;
  ndt_matching::current_velocity_z = 0.0;
  ndt_matching::angular_velocity = 0.0;

  ndt_matching::current_accel = 0.0;
  ndt_matching::current_accel_x = 0.0;
  ndt_matching::current_accel_y = 0.0;
  ndt_matching::current_accel_z = 0.0;

  ndt_matching::offset_x = 0.0;
  ndt_matching::offset_y = 0.0;
  ndt_matching::offset_z = 0.0;
  ndt_matching::offset_yaw = 0.0;

  ndt_matching::offset_imu_x = 0.0;
  ndt_matching::offset_imu_y = 0.0;
  ndt_matching::offset_imu_z = 0.0;
  ndt_matching::offset_imu_roll = 0.0;
  ndt_matching::offset_imu_pitch = 0.0;
  ndt_matching::offset_imu_yaw = 0.0;

  ndt_matching::offset_odom_x = 0.0;
  ndt_matching::offset_odom_y = 0.0;
  ndt_matching::offset_odom_z = 0.0;
  ndt_matching::offset_odom_roll = 0.0;
  ndt_matching::offset_odom_pitch = 0.0;
  ndt_matching::offset_odom_yaw = 0.0;

  ndt_matching::offset_imu_odom_x = 0.0;
  ndt_matching::offset_imu_odom_y = 0.0;
  ndt_matching::offset_imu_odom_z = 0.0;
  ndt_matching::offset_imu_odom_roll = 0.0;
  ndt_matching::offset_imu_odom_pitch = 0.0;
  ndt_matching::offset_imu_odom_yaw = 0.0;

  ndt_matching::init_pos_set = 1;
}

void ndt_matching::imu_odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  ndt_matching::current_pose_imu_odom.roll += diff_imu_roll;
  ndt_matching::current_pose_imu_odom.pitch += diff_imu_pitch;
  ndt_matching::current_pose_imu_odom.yaw += diff_imu_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  ndt_matching::offset_imu_odom_x += diff_distance * cos(-ndt_matching::current_pose_imu_odom.pitch) * cos(ndt_matching::current_pose_imu_odom.yaw);
  ndt_matching::offset_imu_odom_y += diff_distance * cos(-ndt_matching::current_pose_imu_odom.pitch) * sin(ndt_matching::current_pose_imu_odom.yaw);
  ndt_matching::offset_imu_odom_z += diff_distance * sin(-ndt_matching::current_pose_imu_odom.pitch);

  ndt_matching::offset_imu_odom_roll += diff_imu_roll;
  ndt_matching::offset_imu_odom_pitch += diff_imu_pitch;
  ndt_matching::offset_imu_odom_yaw += diff_imu_yaw;

  ndt_matching::predict_pose_imu_odom.x = ndt_matching::previous_pose.x + ndt_matching::offset_imu_odom_x;
  ndt_matching::predict_pose_imu_odom.y = ndt_matching::previous_pose.y + ndt_matching::offset_imu_odom_y;
  ndt_matching::predict_pose_imu_odom.z = ndt_matching::previous_pose.z + ndt_matching::offset_imu_odom_z;
  ndt_matching::predict_pose_imu_odom.roll = ndt_matching::previous_pose.roll + ndt_matching::offset_imu_odom_roll;
  ndt_matching::predict_pose_imu_odom.pitch = ndt_matching::previous_pose.pitch + ndt_matching::offset_imu_odom_pitch;
  ndt_matching::predict_pose_imu_odom.yaw = ndt_matching::previous_pose.yaw + ndt_matching::offset_imu_odom_yaw;

  previous_time = current_time;
}

void ndt_matching::odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  ndt_matching::current_pose_odom.roll += diff_odom_roll;
  ndt_matching::current_pose_odom.pitch += diff_odom_pitch;
  ndt_matching::current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  ndt_matching::offset_odom_x += diff_distance * cos(-ndt_matching::current_pose_odom.pitch) * cos(ndt_matching::current_pose_odom.yaw);
  ndt_matching::offset_odom_y += diff_distance * cos(-ndt_matching::current_pose_odom.pitch) * sin(ndt_matching::current_pose_odom.yaw);
  ndt_matching::offset_odom_z += diff_distance * sin(-ndt_matching::current_pose_odom.pitch);

  ndt_matching::offset_odom_roll += diff_odom_roll;
  ndt_matching::offset_odom_pitch += diff_odom_pitch;
  ndt_matching::offset_odom_yaw += diff_odom_yaw;

  ndt_matching::predict_pose_odom.x = ndt_matching::previous_pose.x + ndt_matching::offset_odom_x;
  ndt_matching::predict_pose_odom.y = ndt_matching::previous_pose.y + ndt_matching::offset_odom_y;
  ndt_matching::predict_pose_odom.z = ndt_matching::previous_pose.z + ndt_matching::offset_odom_z;
  ndt_matching::predict_pose_odom.roll = ndt_matching::previous_pose.roll + ndt_matching::offset_odom_roll;
  ndt_matching::predict_pose_odom.pitch = ndt_matching::previous_pose.pitch + ndt_matching::offset_odom_pitch;
  ndt_matching::predict_pose_odom.yaw = ndt_matching::previous_pose.yaw + ndt_matching::offset_odom_yaw;

  previous_time = current_time;
}

void ndt_matching::imu_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_imu_roll = imu.angular_velocity.x * diff_time;
  double diff_imu_pitch = imu.angular_velocity.y * diff_time;
  double diff_imu_yaw = imu.angular_velocity.z * diff_time;

  ndt_matching::current_pose_imu.roll += diff_imu_roll;
  ndt_matching::current_pose_imu.pitch += diff_imu_pitch;
  ndt_matching::current_pose_imu.yaw += diff_imu_yaw;

  double accX1 = imu.linear_acceleration.x;
  double accY1 = std::cos(ndt_matching::current_pose_imu.roll) * imu.linear_acceleration.y -
                 std::sin(ndt_matching::current_pose_imu.roll) * imu.linear_acceleration.z;
  double accZ1 = std::sin(ndt_matching::current_pose_imu.roll) * imu.linear_acceleration.y +
                 std::cos(ndt_matching::current_pose_imu.roll) * imu.linear_acceleration.z;

  double accX2 = std::sin(ndt_matching::current_pose_imu.pitch) * accZ1 + std::cos(ndt_matching::current_pose_imu.pitch) * accX1;
  double accY2 = accY1;
  double accZ2 = std::cos(ndt_matching::current_pose_imu.pitch) * accZ1 - std::sin(ndt_matching::current_pose_imu.pitch) * accX1;

  double accX = std::cos(ndt_matching::current_pose_imu.yaw) * accX2 - std::sin(ndt_matching::current_pose_imu.yaw) * accY2;
  double accY = std::sin(ndt_matching::current_pose_imu.yaw) * accX2 + std::cos(ndt_matching::current_pose_imu.yaw) * accY2;
  double accZ = accZ2;

  ndt_matching::offset_imu_x += ndt_matching::current_velocity_imu_x * diff_time + accX * diff_time * diff_time / 2.0;
  ndt_matching::offset_imu_y += ndt_matching::current_velocity_imu_y * diff_time + accY * diff_time * diff_time / 2.0;
  ndt_matching::offset_imu_z += ndt_matching::current_velocity_imu_z * diff_time + accZ * diff_time * diff_time / 2.0;

  ndt_matching::current_velocity_imu_x += accX * diff_time;
  ndt_matching::current_velocity_imu_y += accY * diff_time;
  ndt_matching::current_velocity_imu_z += accZ * diff_time;

  ndt_matching::offset_imu_roll += diff_imu_roll;
  ndt_matching::offset_imu_pitch += diff_imu_pitch;
  ndt_matching::offset_imu_yaw += diff_imu_yaw;

  ndt_matching::predict_pose_imu.x = ndt_matching::previous_pose.x + ndt_matching::offset_imu_x;
  ndt_matching::predict_pose_imu.y = ndt_matching::previous_pose.y + ndt_matching::offset_imu_y;
  ndt_matching::predict_pose_imu.z = ndt_matching::previous_pose.z + ndt_matching::offset_imu_z;
  ndt_matching::predict_pose_imu.roll = ndt_matching::previous_pose.roll + ndt_matching::offset_imu_roll;
  ndt_matching::predict_pose_imu.pitch = ndt_matching::previous_pose.pitch + ndt_matching::offset_imu_pitch;
  ndt_matching::predict_pose_imu.yaw = ndt_matching::previous_pose.yaw + ndt_matching::offset_imu_yaw;

  previous_time = current_time;
}

double ndt_matching::wrapToPm(double a_num, const double a_max)
{
  if (a_num >= a_max)
  {
    a_num -= 2.0 * a_max;
  }
  return a_num;
}

double ndt_matching::wrapToPmPi(const double a_angle_rad)
{
  return ndt_matching::wrapToPm(a_angle_rad, M_PI);
}

double ndt_matching::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad >= M_PI)
    diff_rad = diff_rad - 2 * M_PI;
  else if (diff_rad < -M_PI)
    diff_rad = diff_rad + 2 * M_PI;
  return diff_rad;
}

void ndt_matching::odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  // std::cout << __func__ << std::endl;

  odom = *input;
  ndt_matching::odom_calc(input->header.stamp);
}

void ndt_matching::imuUpsideDown(const sensor_msgs::Imu::Ptr input)
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

void ndt_matching::imu_callback(const sensor_msgs::Imu::Ptr& input)
{
  // std::cout << __func__ << std::endl;

  if (ndt_matching::_imu_upside_down)
    ndt_matching::imuUpsideDown(input);

  const ros::Time current_time = input->header.stamp;
  static ros::Time previous_time = current_time;
  const double diff_time = (current_time - previous_time).toSec();

  double imu_roll, imu_pitch, imu_yaw;
  tf::Quaternion imu_orientation;
  tf::quaternionMsgToTF(input->orientation, imu_orientation);
  tf::Matrix3x3(imu_orientation).getRPY(imu_roll, imu_pitch, imu_yaw);

  imu_roll = ndt_matching::wrapToPmPi(imu_roll);
  imu_pitch = ndt_matching::wrapToPmPi(imu_pitch);
  imu_yaw = ndt_matching::wrapToPmPi(imu_yaw);

   double previous_imu_roll = imu_roll, previous_imu_pitch = imu_pitch, previous_imu_yaw = imu_yaw;
  const double diff_imu_roll = ndt_matching::calcDiffForRadian(imu_roll, previous_imu_roll);
  const double diff_imu_pitch = ndt_matching::calcDiffForRadian(imu_pitch, previous_imu_pitch);
  const double diff_imu_yaw = ndt_matching::calcDiffForRadian(imu_yaw, previous_imu_yaw);

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

  ndt_matching::imu_calc(input->header.stamp);

  previous_time = current_time;
  previous_imu_roll = imu_roll;
  previous_imu_pitch = imu_pitch;
  previous_imu_yaw = imu_yaw;
}

void ndt_matching::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  if (ndt_matching::map_loaded == 1 && ndt_matching::init_pos_set == 1)
  {
    ndt_matching::matching_start = std::chrono::system_clock::now();

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion predict_q, ndt_q, current_q, localizer_q;

    pcl::PointXYZ p;
    pcl::PointCloud<pcl::PointXYZ> filtered_scan;

    ros::Time current_scan_time = input->header.stamp;
    static ros::Time previous_scan_time = current_scan_time;

    pcl::fromROSMsg(*input, filtered_scan);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_scan));
    int scan_points_num = filtered_scan_ptr->size();

    Eigen::Matrix4f t(Eigen::Matrix4f::Identity());   // base_link
    Eigen::Matrix4f t2(Eigen::Matrix4f::Identity());  // localizer

    std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start,
        getFitnessScore_end;
     double align_time, getFitnessScore_time = 0.0;

    pthread_mutex_lock(&(ndt_matching::mutex));

    if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
      ndt.setInputSource(filtered_scan_ptr);
    else if (ndt_matching::_method_type == MethodType::PCL_ANH)
      anh_ndt.setInputSource(filtered_scan_ptr);

  #ifdef USE_PCL_OPENMP
    else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
      omp_ndt.setInputSource(filtered_scan_ptr);
  #endif

    // Guess the initial gross estimation of the transformation
    double diff_time = (current_scan_time - previous_scan_time).toSec();

    if (ndt_matching::_offset == "linear")
    {
      ndt_matching::offset_x = ndt_matching::current_velocity_x * diff_time;
      ndt_matching::offset_y = ndt_matching::current_velocity_y * diff_time;
      ndt_matching::offset_z = ndt_matching::current_velocity_z * diff_time;
      ndt_matching::offset_yaw = ndt_matching::angular_velocity * diff_time;
    }
    else if (ndt_matching::_offset == "quadratic")
    {
      ndt_matching::offset_x = (ndt_matching::current_velocity_x + ndt_matching::current_accel_x * diff_time) * diff_time;
      ndt_matching::offset_y = (ndt_matching::current_velocity_y + ndt_matching::current_accel_y * diff_time) * diff_time;
      ndt_matching::offset_z = ndt_matching::current_velocity_z * diff_time;
      ndt_matching::offset_yaw = ndt_matching::angular_velocity * diff_time;
    }
    else if (ndt_matching::_offset == "zero")
    {
      ndt_matching::offset_x = 0.0;
      ndt_matching::offset_y = 0.0;
      ndt_matching::offset_z = 0.0;
      ndt_matching::offset_yaw = 0.0;
    }

    ndt_matching::predict_pose.x = ndt_matching::previous_pose.x + ndt_matching::offset_x;
    ndt_matching::predict_pose.y = ndt_matching::previous_pose.y + ndt_matching::offset_y;
    ndt_matching::predict_pose.z = ndt_matching::previous_pose.z + ndt_matching::offset_z;
    ndt_matching::predict_pose.roll = ndt_matching::previous_pose.roll;
    ndt_matching::predict_pose.pitch = ndt_matching::previous_pose.pitch;
    ndt_matching::predict_pose.yaw = ndt_matching::previous_pose.yaw + ndt_matching::offset_yaw;

    if (ndt_matching::_use_imu == true && ndt_matching::_use_odom == true)
      ndt_matching::imu_odom_calc(current_scan_time);
    if (ndt_matching::_use_imu == true && ndt_matching::_use_odom == false)
      ndt_matching::imu_calc(current_scan_time);
    if (ndt_matching::_use_imu == false && ndt_matching::_use_odom == true)
      ndt_matching::odom_calc(current_scan_time);

    pose predict_pose_for_ndt;
    if (ndt_matching::_use_imu == true && ndt_matching::_use_odom == true)
      predict_pose_for_ndt = ndt_matching::predict_pose_imu_odom;
    else if (ndt_matching::_use_imu == true && ndt_matching::_use_odom == false)
      predict_pose_for_ndt = ndt_matching::predict_pose_imu;
    else if (ndt_matching::_use_imu == false && ndt_matching::_use_odom == true)
      predict_pose_for_ndt = ndt_matching::predict_pose_odom;
    else
      predict_pose_for_ndt = ndt_matching::predict_pose;

    Eigen::Translation3f init_translation(predict_pose_for_ndt.x, predict_pose_for_ndt.y, predict_pose_for_ndt.z);
    Eigen::AngleAxisf init_rotation_x(predict_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(predict_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(predict_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * ndt_matching::tf_btol;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (ndt_matching::_method_type == MethodType::PCL_GENERIC)
    {
      align_start = std::chrono::system_clock::now();
      ndt.align(*output_cloud, init_guess);
      align_end = std::chrono::system_clock::now();

      ndt_matching::has_converged = ndt.hasConverged();

      t = ndt.getFinalTransformation();
      ndt_matching::iteration= ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      ndt_matching::fitness_score = ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      ndt_matching::trans_probability = ndt.getTransformationProbability();
    }
    else if (ndt_matching::_method_type == MethodType::PCL_ANH)
    {
      align_start = std::chrono::system_clock::now();
      anh_ndt.align(init_guess);
      align_end = std::chrono::system_clock::now();

      ndt_matching::has_converged = anh_ndt.hasConverged();

      t = anh_ndt.getFinalTransformation();
      ndt_matching::iteration= anh_ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      ndt_matching::fitness_score = anh_ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      ndt_matching::trans_probability = anh_ndt.getTransformationProbability();
    }

  #ifdef USE_PCL_OPENMP
    else if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
    {
      align_start = std::chrono::system_clock::now();
      omp_ndt.align(*output_cloud, init_guess);
      align_end = std::chrono::system_clock::now();

      ndt_matching::has_converged = omp_ndt.hasConverged();

      t = omp_ndt.getFinalTransformation();
      ndt_matching::iteration= omp_ndt.getFinalNumIteration();

      getFitnessScore_start = std::chrono::system_clock::now();
      ndt_matching::fitness_score = omp_ndt.getFitnessScore();
      getFitnessScore_end = std::chrono::system_clock::now();

      ndt_matching::trans_probability = omp_ndt.getTransformationProbability();
    }
  #endif
    align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

    t2 = t * ndt_matching::tf_btol.inverse();

    getFitnessScore_time =
        std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() /
        1000.0;

    pthread_mutex_unlock(&(ndt_matching::mutex));

    tf::Matrix3x3 mat_l;  // localizer
    mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
                   static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                   static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

    // Update ndt_matching::localizer_pose
    ndt_matching::localizer_pose.x = t(0, 3);
    ndt_matching::localizer_pose.y = t(1, 3);
    ndt_matching::localizer_pose.z = t(2, 3);
    mat_l.getRPY(ndt_matching::localizer_pose.roll, ndt_matching::localizer_pose.pitch, ndt_matching::localizer_pose.yaw, 1);

    tf::Matrix3x3 mat_b;  // base_link
    mat_b.setValue(static_cast<double>(t2(0, 0)), static_cast<double>(t2(0, 1)), static_cast<double>(t2(0, 2)),
                   static_cast<double>(t2(1, 0)), static_cast<double>(t2(1, 1)), static_cast<double>(t2(1, 2)),
                   static_cast<double>(t2(2, 0)), static_cast<double>(t2(2, 1)), static_cast<double>(t2(2, 2)));

    // Update ndt_pose
    ndt_matching::ndt_pose.x = t2(0, 3);
    ndt_matching::ndt_pose.y = t2(1, 3);
    ndt_matching::ndt_pose.z = t2(2, 3);
    mat_b.getRPY(ndt_matching::ndt_pose.roll, ndt_matching::ndt_pose.pitch, ndt_matching::ndt_pose.yaw, 1);

    // Calculate the difference between ndt_matching::ndt_pose and ndt_matching::predict_pose
    ndt_matching::predict_pose_error = sqrt((ndt_matching::ndt_pose.x - predict_pose_for_ndt.x) * (ndt_matching::ndt_pose.x - predict_pose_for_ndt.x) +
                              (ndt_matching::ndt_pose.y - predict_pose_for_ndt.y) * (ndt_matching::ndt_pose.y - predict_pose_for_ndt.y) +
                              (ndt_matching::ndt_pose.z - predict_pose_for_ndt.z) * (ndt_matching::ndt_pose.z - predict_pose_for_ndt.z));

    ndt_matching::pred_error.data = ndt_matching::predict_pose_error;
    rmse_pub.publish(ndt_matching::pred_error);

    if (ndt_matching::predict_pose_error <= PREDICT_POSE_THRESHOLD)
    {
      use_predict_pose = 0;
    }
    else
    {
      use_predict_pose = 1;
    }
    use_predict_pose = 0;

    if (use_predict_pose == 0)
    {
      ndt_matching::current_pose.x = ndt_matching::ndt_pose.x;
      ndt_matching::current_pose.y = ndt_matching::ndt_pose.y;
      ndt_matching::current_pose.z = ndt_matching::ndt_pose.z;
      ndt_matching::current_pose.roll = ndt_matching::ndt_pose.roll;
      ndt_matching::current_pose.pitch = ndt_matching::ndt_pose.pitch;
      ndt_matching::current_pose.yaw = ndt_matching::ndt_pose.yaw;
    }
    else
    {
      ndt_matching::current_pose.x = predict_pose_for_ndt.x;
      ndt_matching::current_pose.y = predict_pose_for_ndt.y;
      ndt_matching::current_pose.z = predict_pose_for_ndt.z;
      ndt_matching::current_pose.roll = predict_pose_for_ndt.roll;
      ndt_matching::current_pose.pitch = predict_pose_for_ndt.pitch;
      ndt_matching::current_pose.yaw = predict_pose_for_ndt.yaw;
    }

    // Compute the velocity and acceleration
    ndt_matching::diff_x = ndt_matching::current_pose.x - ndt_matching::previous_pose.x;
    ndt_matching::diff_y = ndt_matching::current_pose.y - ndt_matching::previous_pose.y;
    ndt_matching::diff_z = ndt_matching::current_pose.z - ndt_matching::previous_pose.z;
    ndt_matching::diff_yaw = ndt_matching::calcDiffForRadian(ndt_matching::current_pose.yaw, ndt_matching::previous_pose.yaw);
    ndt_matching::diff = sqrt(ndt_matching::diff_x * ndt_matching::diff_x + ndt_matching::diff_y * ndt_matching::diff_y + ndt_matching::diff_z * ndt_matching::diff_z);

    ndt_matching::current_velocity = (diff_time > 0) ? (ndt_matching::diff / diff_time) : 0;
    std::cout << "DEBUGGING2: diff_time = " << diff_time << ": diff = " << ndt_matching::diff << std::endl;
    ndt_matching::current_velocity_x = (diff_time > 0) ? (ndt_matching::diff_x / diff_time) : 0;
    ndt_matching::current_velocity_y = (diff_time > 0) ? (ndt_matching::diff_y / diff_time) : 0;
    ndt_matching::current_velocity_z = (diff_time > 0) ? (ndt_matching::diff_z / diff_time) : 0;
    ndt_matching::angular_velocity = (diff_time > 0) ? (ndt_matching::diff_yaw / diff_time) : 0;

    ndt_matching::current_pose_imu.x = ndt_matching::current_pose.x;
    ndt_matching::current_pose_imu.y = ndt_matching::current_pose.y;
    ndt_matching::current_pose_imu.z = ndt_matching::current_pose.z;
    ndt_matching::current_pose_imu.roll = ndt_matching::current_pose.roll;
    ndt_matching::current_pose_imu.pitch = ndt_matching::current_pose.pitch;
    ndt_matching::current_pose_imu.yaw = ndt_matching::current_pose.yaw;

    ndt_matching::current_velocity_imu_x = ndt_matching::current_velocity_x;
    ndt_matching::current_velocity_imu_y = ndt_matching::current_velocity_y;
    ndt_matching::current_velocity_imu_z = ndt_matching::current_velocity_z;

    ndt_matching::current_pose_odom.x = ndt_matching::current_pose.x;
    ndt_matching::current_pose_odom.y = ndt_matching::current_pose.y;
    ndt_matching::current_pose_odom.z = ndt_matching::current_pose.z;
    ndt_matching::current_pose_odom.roll = ndt_matching::current_pose.roll;
    ndt_matching::current_pose_odom.pitch = ndt_matching::current_pose.pitch;
    ndt_matching::current_pose_odom.yaw = ndt_matching::current_pose.yaw;

    ndt_matching::current_pose_imu_odom.x = ndt_matching::current_pose.x;
    ndt_matching::current_pose_imu_odom.y = ndt_matching::current_pose.y;
    ndt_matching::current_pose_imu_odom.z = ndt_matching::current_pose.z;
    ndt_matching::current_pose_imu_odom.roll = ndt_matching::current_pose.roll;
    ndt_matching::current_pose_imu_odom.pitch = ndt_matching::current_pose.pitch;
    ndt_matching::current_pose_imu_odom.yaw = ndt_matching::current_pose.yaw;

    ndt_matching::current_velocity_smooth = (ndt_matching::current_velocity +ndt_matching::previous_velocity + ndt_matching::previous_previous_velocity) / 3.0;
    if (ndt_matching::current_velocity_smooth < 0.2)
    {
      ndt_matching::current_velocity_smooth = 0.0;
    }

    ndt_matching::current_accel = (diff_time > 0) ? ((ndt_matching::current_velocity -ndt_matching::previous_velocity) / diff_time) : 0;
    ndt_matching::current_accel_x = (diff_time > 0) ? ((ndt_matching::current_velocity_x -ndt_matching::previous_velocity_x) / diff_time) : 0;
    ndt_matching::current_accel_y = (diff_time > 0) ? ((ndt_matching::current_velocity_y -ndt_matching::previous_velocity_y) / diff_time) : 0;
    ndt_matching::current_accel_z = (diff_time > 0) ? ((ndt_matching::current_velocity_z -ndt_matching::previous_velocity_z) / diff_time) : 0;

    ndt_matching::estimated_vel_mps.data = ndt_matching::current_velocity;
    ndt_matching::estimated_vel_kmph.data = ndt_matching::current_velocity * 3.6;

    ndt_matching::estimated_vel_mps_pub.publish(ndt_matching::estimated_vel_mps);
    ndt_matching::estimated_vel_kmph_pub.publish(ndt_matching::estimated_vel_kmph);

    // Set values for publishing pose
    predict_q.setRPY(ndt_matching::predict_pose.roll, ndt_matching::predict_pose.pitch, ndt_matching::predict_pose.yaw);
    if (ndt_matching::_use_local_transform == true)
    {
      tf::Vector3 v(ndt_matching::predict_pose.x, ndt_matching::predict_pose.y, ndt_matching::predict_pose.z);
      tf::Transform transform(predict_q, v);
      ndt_matching::predict_pose_msg.header.frame_id = "/map";
      ndt_matching::predict_pose_msg.header.stamp = current_scan_time;
      ndt_matching::predict_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      ndt_matching::predict_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      ndt_matching::predict_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      ndt_matching::predict_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      ndt_matching::predict_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      ndt_matching::predict_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      ndt_matching::predict_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      ndt_matching::predict_pose_msg.header.frame_id = "/map";
      ndt_matching::predict_pose_msg.header.stamp = current_scan_time;
      ndt_matching::predict_pose_msg.pose.position.x = ndt_matching::predict_pose.x;
      ndt_matching::predict_pose_msg.pose.position.y = ndt_matching::predict_pose.y;
      ndt_matching::predict_pose_msg.pose.position.z = ndt_matching::predict_pose.z;
      ndt_matching::predict_pose_msg.pose.orientation.x = predict_q.x();
      ndt_matching::predict_pose_msg.pose.orientation.y = predict_q.y();
      ndt_matching::predict_pose_msg.pose.orientation.z = predict_q.z();
      ndt_matching::predict_pose_msg.pose.orientation.w = predict_q.w();
    }

    tf::Quaternion predict_q_imu;
    predict_q_imu.setRPY(ndt_matching::predict_pose_imu.roll, ndt_matching::predict_pose_imu.pitch, ndt_matching::predict_pose_imu.yaw);
    ndt_matching::predict_pose_imu_msg.header.frame_id = "map";
    ndt_matching::predict_pose_imu_msg.header.stamp = input->header.stamp;
    ndt_matching::predict_pose_imu_msg.pose.position.x = ndt_matching::predict_pose_imu.x;
    ndt_matching::predict_pose_imu_msg.pose.position.y = ndt_matching::predict_pose_imu.y;
    ndt_matching::predict_pose_imu_msg.pose.position.z = ndt_matching::predict_pose_imu.z;
    ndt_matching::predict_pose_imu_msg.pose.orientation.x = predict_q_imu.x();
    ndt_matching::predict_pose_imu_msg.pose.orientation.y = predict_q_imu.y();
    ndt_matching::predict_pose_imu_msg.pose.orientation.z = predict_q_imu.z();
    ndt_matching::predict_pose_imu_msg.pose.orientation.w = predict_q_imu.w();
    ndt_matching::predict_pose_imu_pub.publish(ndt_matching::predict_pose_imu_msg);

    tf::Quaternion predict_q_odom;
    predict_q_odom.setRPY(ndt_matching::predict_pose_odom.roll, ndt_matching::predict_pose_odom.pitch, ndt_matching::predict_pose_odom.yaw);
    ndt_matching::predict_pose_odom_msg.header.frame_id = "map";
    ndt_matching::predict_pose_odom_msg.header.stamp = input->header.stamp;
    ndt_matching::predict_pose_odom_msg.pose.position.x = ndt_matching::predict_pose_odom.x;
    ndt_matching::predict_pose_odom_msg.pose.position.y = ndt_matching::predict_pose_odom.y;
    ndt_matching::predict_pose_odom_msg.pose.position.z = ndt_matching::predict_pose_odom.z;
    ndt_matching::predict_pose_odom_msg.pose.orientation.x = predict_q_odom.x();
    ndt_matching::predict_pose_odom_msg.pose.orientation.y = predict_q_odom.y();
    ndt_matching::predict_pose_odom_msg.pose.orientation.z = predict_q_odom.z();
    ndt_matching::predict_pose_odom_msg.pose.orientation.w = predict_q_odom.w();
    ndt_matching::predict_pose_odom_pub.publish(ndt_matching::predict_pose_odom_msg);

    tf::Quaternion predict_q_imu_odom;
    predict_q_imu_odom.setRPY(ndt_matching::predict_pose_imu_odom.roll, ndt_matching::predict_pose_imu_odom.pitch, ndt_matching::predict_pose_imu_odom.yaw);
    ndt_matching::predict_pose_imu_odom_msg.header.frame_id = "map";
    ndt_matching::predict_pose_imu_odom_msg.header.stamp = input->header.stamp;
    ndt_matching::predict_pose_imu_odom_msg.pose.position.x = ndt_matching::predict_pose_imu_odom.x;
    ndt_matching::predict_pose_imu_odom_msg.pose.position.y = ndt_matching::predict_pose_imu_odom.y;
    ndt_matching::predict_pose_imu_odom_msg.pose.position.z = ndt_matching::predict_pose_imu_odom.z;
    ndt_matching::predict_pose_imu_odom_msg.pose.orientation.x = predict_q_imu_odom.x();
    ndt_matching::predict_pose_imu_odom_msg.pose.orientation.y = predict_q_imu_odom.y();
    ndt_matching::predict_pose_imu_odom_msg.pose.orientation.z = predict_q_imu_odom.z();
    ndt_matching::predict_pose_imu_odom_msg.pose.orientation.w = predict_q_imu_odom.w();
    ndt_matching::predict_pose_imu_odom_pub.publish(ndt_matching::predict_pose_imu_odom_msg);

    ndt_q.setRPY(ndt_matching::ndt_pose.roll, ndt_matching::ndt_pose.pitch, ndt_matching::ndt_pose.yaw);
    if (ndt_matching::_use_local_transform == true)
    {
      tf::Vector3 v(ndt_matching::ndt_pose.x, ndt_matching::ndt_pose.y, ndt_matching::ndt_pose.z);
      tf::Transform transform(ndt_q, v);
      ndt_matching::ndt_pose_msg.header.frame_id = "/map";
      ndt_matching::ndt_pose_msg.header.stamp = current_scan_time;
      ndt_matching::ndt_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      ndt_matching::ndt_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      ndt_matching::ndt_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      ndt_matching::ndt_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      ndt_matching::ndt_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      ndt_matching::ndt_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      ndt_matching::ndt_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      ndt_matching::ndt_pose_msg.header.frame_id = "/map";
      ndt_matching::ndt_pose_msg.header.stamp = current_scan_time;
      ndt_matching::ndt_pose_msg.pose.position.x = ndt_matching::ndt_pose.x;
      ndt_matching::ndt_pose_msg.pose.position.y = ndt_matching::ndt_pose.y;
      ndt_matching::ndt_pose_msg.pose.position.z = ndt_matching::ndt_pose.z;
      ndt_matching::ndt_pose_msg.pose.orientation.x = ndt_q.x();
      ndt_matching::ndt_pose_msg.pose.orientation.y = ndt_q.y();
      ndt_matching::ndt_pose_msg.pose.orientation.z = ndt_q.z();
      ndt_matching::ndt_pose_msg.pose.orientation.w = ndt_q.w();
    }

    current_q.setRPY(ndt_matching::current_pose.roll, ndt_matching::current_pose.pitch, ndt_matching::current_pose.yaw);
    // ndt_matching::current_pose is published by vel_pose_mux
    /*
    ndt_matching::current_pose_msg.header.frame_id = "/map";
    ndt_matching::current_pose_msg.header.stamp = current_scan_time;
    ndt_matching::current_pose_msg.pose.position.x = ndt_matching::current_pose.x;
    ndt_matching::current_pose_msg.pose.position.y = ndt_matching::current_pose.y;
    ndt_matching::current_pose_msg.pose.position.z = ndt_matching::current_pose.z;
    ndt_matching::current_pose_msg.pose.orientation.x = current_q.x();
    ndt_matching::current_pose_msg.pose.orientation.y = current_q.y();
    ndt_matching::current_pose_msg.pose.orientation.z = current_q.z();
    ndt_matching::current_pose_msg.pose.orientation.w = current_q.w();
    */

    localizer_q.setRPY(ndt_matching::localizer_pose.roll, ndt_matching::localizer_pose.pitch, ndt_matching::localizer_pose.yaw);
    if (ndt_matching::_use_local_transform == true)
    {
      tf::Vector3 v(ndt_matching::localizer_pose.x, ndt_matching::localizer_pose.y, ndt_matching::localizer_pose.z);
      tf::Transform transform(localizer_q, v);
      ndt_matching::localizer_pose_msg.header.frame_id = "/map";
      ndt_matching::localizer_pose_msg.header.stamp = current_scan_time;
      ndt_matching::localizer_pose_msg.pose.position.x = (local_transform * transform).getOrigin().getX();
      ndt_matching::localizer_pose_msg.pose.position.y = (local_transform * transform).getOrigin().getY();
      ndt_matching::localizer_pose_msg.pose.position.z = (local_transform * transform).getOrigin().getZ();
      ndt_matching::localizer_pose_msg.pose.orientation.x = (local_transform * transform).getRotation().x();
      ndt_matching::localizer_pose_msg.pose.orientation.y = (local_transform * transform).getRotation().y();
      ndt_matching::localizer_pose_msg.pose.orientation.z = (local_transform * transform).getRotation().z();
      ndt_matching::localizer_pose_msg.pose.orientation.w = (local_transform * transform).getRotation().w();
    }
    else
    {
      ndt_matching::localizer_pose_msg.header.frame_id = "/map";
      ndt_matching::localizer_pose_msg.header.stamp = current_scan_time;
      ndt_matching::localizer_pose_msg.pose.position.x = ndt_matching::localizer_pose.x;
      ndt_matching::localizer_pose_msg.pose.position.y = ndt_matching::localizer_pose.y;
      ndt_matching::localizer_pose_msg.pose.position.z = ndt_matching::localizer_pose.z;
      ndt_matching::localizer_pose_msg.pose.orientation.x = localizer_q.x();
      ndt_matching::localizer_pose_msg.pose.orientation.y = localizer_q.y();
      ndt_matching::localizer_pose_msg.pose.orientation.z = localizer_q.z();
      ndt_matching::localizer_pose_msg.pose.orientation.w = localizer_q.w();
    }

    ndt_matching::predict_pose_pub.publish(ndt_matching::predict_pose_msg);
    ndt_matching::ndt_pose_pub.publish(ndt_matching::ndt_pose_msg);
    // ndt_matching::current_pose is published by vel_pose_mux
    //    ndt_matching::current_pose_pub.publish(ndt_matching::current_pose_msg);
    ndt_matching::localizer_pose_pub.publish(ndt_matching::localizer_pose_msg);

    // Send TF "/base_link" to "/map"
    transform.setOrigin(tf::Vector3(ndt_matching::current_pose.x, ndt_matching::current_pose.y, ndt_matching::current_pose.z));
    transform.setRotation(current_q);
    //    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));
    if (ndt_matching::_use_local_transform == true)
    {
      br.sendTransform(tf::StampedTransform(local_transform * transform, current_scan_time, "/map", "/base_link"));
    }
    else
    {
      br.sendTransform(tf::StampedTransform(transform, current_scan_time, "/map", "/base_link"));
    }

    ndt_matching::matching_end = std::chrono::system_clock::now();
    ndt_matching::exe_time = std::chrono::duration_cast<std::chrono::microseconds>(ndt_matching::matching_end - ndt_matching::matching_start).count() / 1000.0;
    ndt_matching::time_ndt_matching.data = ndt_matching::exe_time;
    ndt_matching::time_ndt_matching_pub.publish(ndt_matching::time_ndt_matching);

    // Set values for /estimate_twist
    ndt_matching::estimate_twist_msg.header.stamp = current_scan_time;
    ndt_matching::estimate_twist_msg.header.frame_id = "/base_link";
    ndt_matching::estimate_twist_msg.twist.linear.x = ndt_matching::current_velocity;
    ndt_matching::estimate_twist_msg.twist.linear.y = 0.0;
    ndt_matching::estimate_twist_msg.twist.linear.z = 0.0;
    ndt_matching::estimate_twist_msg.twist.angular.x = 0.0;
    ndt_matching::estimate_twist_msg.twist.angular.y = 0.0;
    ndt_matching::estimate_twist_msg.twist.angular.z = ndt_matching::angular_velocity;

    ndt_matching::estimate_twist_pub.publish(ndt_matching::estimate_twist_msg);

    geometry_msgs::Vector3Stamped estimate_vel_msg;
    estimate_vel_msg.header.stamp = current_scan_time;
    estimate_vel_msg.vector.x = ndt_matching::current_velocity;
    ndt_matching::estimated_vel_pub.publish(estimate_vel_msg);

    // Set values for /ndt_stat
    ndt_matching::ndt_stat_msg.header.stamp = current_scan_time;
    ndt_matching::ndt_stat_msg.exe_time = time_ndt_matching.data;
    ndt_matching::ndt_stat_msg.iteration= iteration;
    ndt_matching::ndt_stat_msg.score = ndt_matching::fitness_score;
    ndt_matching::ndt_stat_msg.velocity = ndt_matching::current_velocity;
    ndt_matching::ndt_stat_msg.acceleration = ndt_matching::current_accel;
    ndt_matching::ndt_stat_msg.use_predict_pose = 0;

    ndt_matching::ndt_stat_pub.publish(ndt_matching::ndt_stat_msg);
    /* Compute ndt_matching::ndt_reliability */
    ndt_matching::ndt_reliability.data = Wa * (ndt_matching::exe_time / 100.0) * 100.0 + Wb * (ndt_matching::iteration/ 10.0) * 100.0 +
                           Wc * ((2.0 - ndt_matching::trans_probability) / 2.0) * 100.0;
    ndt_matching::ndt_reliability_pub.publish(ndt_matching::ndt_reliability);

    // Write log
    if(ndt_matching::_output_log_data)
    {
      if (!ndt_matching::ofs)
      {
        std::cerr << "Could not open " << ndt_matching::filename << "." << std::endl;
      }
      else
      {
        ndt_matching::ofs << input->header.seq << "," << scan_points_num << "," << ndt_matching::step_size << "," << ndt_matching::trans_eps << "," << std::fixed
            << std::setprecision(5) << ndt_matching::current_pose.x << "," << std::fixed << std::setprecision(5) << ndt_matching::current_pose.y << ","
            << std::fixed << std::setprecision(5) << ndt_matching::current_pose.z << "," << ndt_matching::current_pose.roll << "," << ndt_matching::current_pose.pitch
            << "," << ndt_matching::current_pose.yaw << "," << ndt_matching::predict_pose.x << "," << ndt_matching::predict_pose.y << "," << ndt_matching::predict_pose.z << ","
            << ndt_matching::predict_pose.roll << "," << ndt_matching::predict_pose.pitch << "," << ndt_matching::predict_pose.yaw << ","
            << ndt_matching::current_pose.x - ndt_matching::predict_pose.x << "," << ndt_matching::current_pose.y - ndt_matching::predict_pose.y << ","
            << ndt_matching::current_pose.z - ndt_matching::predict_pose.z << "," << ndt_matching::current_pose.roll - ndt_matching::predict_pose.roll << ","
            << ndt_matching::current_pose.pitch - ndt_matching::predict_pose.pitch << "," << ndt_matching::current_pose.yaw - ndt_matching::predict_pose.yaw << ","
            << ndt_matching::predict_pose_error << "," << ndt_matching::iteration<< "," << ndt_matching::fitness_score << "," << ndt_matching::trans_probability << ","
            << ndt_matching::ndt_reliability.data << "," << ndt_matching::current_velocity << "," << ndt_matching::current_velocity_smooth << "," << ndt_matching::current_accel
            << "," << ndt_matching::angular_velocity << "," << ndt_matching::time_ndt_matching.data << "," << align_time << "," << getFitnessScore_time
            << std::endl;
      }
    }

   /* std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Sequence: " << input->header.seq << std::endl;
    std::cout << "Timestamp: " << input->header.stamp << std::endl;
    std::cout << "Frame ID: " << input->header.frame_id << std::endl;
    //		std::cout << "Number of Scan Points: " << scan_ptr->size() << " points." << std::endl;
    std::cout << "Number of Filtered Scan Points: " << scan_points_num << " points." << std::endl;
    std::cout << "NDT has converged: " << ndt_matching::has_converged << std::endl;
    std::cout << "Fitness Score: " << ndt_matching::fitness_score << std::endl;
    std::cout << "Transformation Probability: " << ndt_matching::trans_probability << std::endl;
    std::cout << "Execution Time: " << ndt_matching::exe_time << " ms." << std::endl;
    std::cout << "Number of Iterations: " << ndt_matching::iteration<< std::endl;
    std::cout << "NDT Reliability: " << ndt_matching::ndt_reliability.data << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
    std::cout << "(" << ndt_matching::current_pose.x << ", " << ndt_matching::current_pose.y << ", " << ndt_matching::current_pose.z << ", " << ndt_matching::current_pose.roll
              << ", " << ndt_matching::current_pose.pitch << ", " << ndt_matching::current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix: " << std::endl;
    std::cout << t << std::endl;
    std::cout << "Align time: " << align_time << std::endl;
    std::cout << "Get fitness score time: " << getFitnessScore_time << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;*/

    ndt_matching::offset_imu_x = 0.0;
    ndt_matching::offset_imu_y = 0.0;
    ndt_matching::offset_imu_z = 0.0;
    ndt_matching::offset_imu_roll = 0.0;
    ndt_matching::offset_imu_pitch = 0.0;
    ndt_matching::offset_imu_yaw = 0.0;

    ndt_matching::offset_odom_x = 0.0;
    ndt_matching::offset_odom_y = 0.0;
    ndt_matching::offset_odom_z = 0.0;
    ndt_matching::offset_odom_roll = 0.0;
    ndt_matching::offset_odom_pitch = 0.0;
    ndt_matching::offset_odom_yaw = 0.0;

    ndt_matching::offset_imu_odom_x = 0.0;
    ndt_matching::offset_imu_odom_y = 0.0;
    ndt_matching::offset_imu_odom_z = 0.0;
    ndt_matching::offset_imu_odom_roll = 0.0;
    ndt_matching::offset_imu_odom_pitch = 0.0;
    ndt_matching::offset_imu_odom_yaw = 0.0;

    // Update previous_***
    ndt_matching::previous_pose.x = ndt_matching::current_pose.x;
    ndt_matching::previous_pose.y = ndt_matching::current_pose.y;
    ndt_matching::previous_pose.z = ndt_matching::current_pose.z;
    ndt_matching::previous_pose.roll = ndt_matching::current_pose.roll;
    ndt_matching::previous_pose.pitch = ndt_matching::current_pose.pitch;
    ndt_matching::previous_pose.yaw = ndt_matching::current_pose.yaw;

    previous_scan_time = current_scan_time;

    ndt_matching::previous_previous_velocity =ndt_matching::previous_velocity;
    ndt_matching::previous_velocity = ndt_matching::current_velocity;
    ndt_matching::previous_velocity_x = ndt_matching::current_velocity_x;
    ndt_matching::previous_velocity_y = ndt_matching::current_velocity_y;
    ndt_matching::previous_velocity_z = ndt_matching::current_velocity_z;
    ndt_matching::previous_accel = ndt_matching::current_accel;

    ndt_matching::previous_estimated_vel_kmph.data = ndt_matching::estimated_vel_kmph.data;
  }
}

void* ndt_matching::thread_func(void* args)
{
  ros::NodeHandle nh_map;
  ros::CallbackQueue map_callback_queue;
  nh_map.setCallbackQueue(&map_callback_queue);

  ros::Subscriber map_sub = nh_map.subscribe("points_map", 10, &ndt_matching::map_callback, this);
  ros::Rate ros_rate(10);
  while (nh_map.ok())
  {
    map_callback_queue.callAvailable(ros::WallDuration());
    ros_rate.sleep();
  }

  return nullptr;
}

typedef void * (*THREADFUNCPTR)(void *);

int ndt_matching::RUN(int argc, char** argv)
{
  ros::init(argc, argv, "ndt_matching");
  pthread_mutex_init(&mutex, NULL);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Set log file name.
  private_nh.getParam("output_log_data", ndt_matching::_output_log_data);
  if(ndt_matching::_output_log_data)
  {
    char buffer[80];
    std::time_t now = std::time(NULL);
    std::tm* pnow = std::localtime(&now);
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    std::string directory_name = "/tmp/aslan/log/ndt_matching";
    ndt_matching::filename = directory_name + "/" + std::string(buffer) + ".csv";
    boost::filesystem::create_directories(boost::filesystem::path(directory_name));
    ndt_matching::ofs.open(ndt_matching::filename.c_str(), std::ios::app);
  }

  // Geting parameters
  int method_type_tmp = 0;

  private_nh.getParam("method_type", method_type_tmp);
  ndt_matching::_method_type = static_cast<MethodType>(method_type_tmp);
  private_nh.getParam("use_gnss", ndt_matching::_use_gnss);
  private_nh.getParam("queue_size", ndt_matching::_queue_size);
  private_nh.getParam("offset", ndt_matching::_offset);
  private_nh.getParam("get_height", ndt_matching::_get_height);
  private_nh.getParam("use_local_transform", ndt_matching::_use_local_transform);
  private_nh.getParam("use_imu", ndt_matching::_use_imu);
  private_nh.getParam("use_odom", ndt_matching::_use_odom);
  private_nh.getParam("imu_upside_down", ndt_matching::_imu_upside_down);
  private_nh.getParam("imu_topic", ndt_matching::_imu_topic);


  if (nh.getParam("localizer", ndt_matching::_localizer) == false)
  {
    std::cout << "localizer is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_x", ndt_matching::_tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_y", ndt_matching::_tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_z", ndt_matching::_tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_roll", ndt_matching::_tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_pitch", ndt_matching::_tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    return 1;
  }
  if (nh.getParam("tf_yaw", ndt_matching::_tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    return 1;
  }

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Log file: " << ndt_matching::filename << std::endl;
  std::cout << "method_type: " << static_cast<int>(ndt_matching::_method_type) << std::endl;
  std::cout << "use_gnss: " << ndt_matching::_use_gnss << std::endl;
  std::cout << "queue_size: " << ndt_matching::_queue_size << std::endl;
  std::cout << "offset: " << ndt_matching::_offset << std::endl;
  std::cout << "get_height: " << ndt_matching::_get_height << std::endl;
  std::cout << "use_local_transform: " << ndt_matching::_use_local_transform << std::endl;
  std::cout << "use_odom: " << ndt_matching::_use_odom << std::endl;
  std::cout << "use_imu: " << ndt_matching::_use_imu << std::endl;
  std::cout << "imu_upside_down: " << ndt_matching::_imu_upside_down << std::endl;
  std::cout << "imu_topic: " << ndt_matching::_imu_topic << std::endl;
  std::cout << "localizer: " << ndt_matching::_localizer << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << ndt_matching::_tf_x << ", " << ndt_matching::_tf_y << ", " << ndt_matching::_tf_z << ", "
            << ndt_matching::_tf_roll << ", " << ndt_matching::_tf_pitch << ", " << ndt_matching::_tf_yaw << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;


  #ifndef USE_PCL_OPENMP
  if (ndt_matching::_method_type == MethodType::PCL_OPENMP)
  {
    std::cerr << "**************************************************************" << std::endl;
    std::cerr << "[ERROR]PCL_OPENMP is not built. Please use other method type." << std::endl;
    std::cerr << "**************************************************************" << std::endl;
    exit(1);
  }
  #endif

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  ndt_matching::tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  // Updated in initialpose_callback or gnss_callback
  ndt_matching::initial_pose.x = 0.0;
  ndt_matching::initial_pose.y = 0.0;
  ndt_matching::initial_pose.z = 0.0;
  ndt_matching::initial_pose.roll = 0.0;
  ndt_matching::initial_pose.pitch = 0.0;
  ndt_matching::initial_pose.yaw = 0.0;

  // Publishers
  ndt_matching::predict_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose", 10);
  ndt_matching::predict_pose_imu_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu", 10);
  ndt_matching::predict_pose_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_odom", 10);
  ndt_matching::predict_pose_imu_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/predict_pose_imu_odom", 10);
  ndt_matching::ndt_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 10);
  // ndt_matching::current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_matching::current_pose", 10);
  ndt_matching::localizer_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 10);
  ndt_matching::estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);
  ndt_matching::estimated_vel_mps_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_mps", 10);
  ndt_matching::estimated_vel_kmph_pub = nh.advertise<std_msgs::Float32>("/estimated_vel_kmph", 10);
  ndt_matching::estimated_vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 10);
  ndt_matching::time_ndt_matching_pub = nh.advertise<std_msgs::Float32>("/time_ndt_matching", 10);
  ndt_matching::ndt_stat_pub = nh.advertise<aslan_msgs::NDTStat>("/ndt_stat", 10);
  ndt_matching::ndt_reliability_pub = nh.advertise<std_msgs::Float32>("/ndt_reliability", 10);
  rmse_pub = nh.advertise<std_msgs::Float64>("/rmse", 10);

  //Subscribers
  ros::Subscriber gnss_sub = nh.subscribe("gnss_pose", 10, &ndt_matching::gnss_callback, this);
  ros::Subscriber param_sub = nh.subscribe("config/ndt", 10, &ndt_matching::param_callback, this);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 10, &ndt_matching::initialpose_callback, this);
  ros::Subscriber points_sub = nh.subscribe("filtered_points", ndt_matching::_queue_size, &ndt_matching::points_callback, this);
  ros::Subscriber odom_sub = nh.subscribe("/vehicle/odom", ndt_matching::_queue_size * 10, &ndt_matching::odom_callback, this);
  ros::Subscriber imu_sub = nh.subscribe(ndt_matching::_imu_topic.c_str(), ndt_matching::_queue_size * 10, &ndt_matching::imu_callback, this);

  pthread_t thread;
  pthread_create(&thread, NULL, (THREADFUNCPTR) &ndt_matching::thread_func, this);

  ros::spin();

  return 0;
}
