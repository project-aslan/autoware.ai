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
#include "waypoint_follower/twist_filter.h"

void twist_filter::removedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    twist_filter::obst_removed.data = msg->data;
    if (twist_filter::obst_removed.data == true){
        twist_filter::zero_twist_counter = 0;
    }
}

void twist_filter::TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{

  double v = msg->twist.linear.x;
  double omega = msg->twist.angular.z;

  if (v == 0.0){
      twist_filter::zero_twist_counter = twist_filter::zero_twist_counter +1;
  }

  if (twist_filter::zero_twist_counter > 0 ){
      v = 0.0;
  }

  if(fabs(omega) < twist_filter::ERROR){
    twist_filter::g_twist_pub.publish(*msg);
    return;
  }

  double max_v = twist_filter::g_lateral_accel_limit / fabs(omega);

  geometry_msgs::TwistStamped tp;
  tp.header = msg->header;

  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);


  tp.twist.linear.x = fabs(a) > twist_filter::g_lateral_accel_limit ? max_v
                    : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  lowpass_linear_x = twist_filter::g_lowpass_gain_linear_x * lowpass_linear_x + (1 - twist_filter::g_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = twist_filter::g_lowpass_gain_angular_z * lowpass_angular_z + (1 - twist_filter::g_lowpass_gain_angular_z) * tp.twist.angular.z;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

    if (twist_filter::zero_twist_counter > 0 ){
        tp.twist.linear.x = 0.0;
    }

  ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  twist_filter::g_twist_pub.publish(tp);

}

void twist_filter::param_callback(const aslan_msgs::ConfigTwistFilter::ConstPtr& config)
{
    twist_filter::g_lateral_accel_limit = config->lateral_accel_limit;
    ROS_INFO("twist_filter::g_lateral_accel_limit = %lf",twist_filter::g_lateral_accel_limit);
    twist_filter::g_lowpass_gain_linear_x = config->lowpass_gain_linear_x;
    ROS_INFO("lowpass_gain_linear_x = %lf",twist_filter::g_lowpass_gain_linear_x);
    twist_filter::g_lowpass_gain_angular_z = config->lowpass_gain_angular_z;
    ROS_INFO("lowpass_gain_angular_z = %lf",twist_filter::g_lowpass_gain_angular_z);

}


int twist_filter::RUN(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber obstacle_removed_sub = nh.subscribe("obstacle_removed", 100, &twist_filter::removedCallback, this);
    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, &twist_filter::TwistCmdCallback, this);
    ros::Subscriber param_sub = nh.subscribe("config/twist_filter", 10, &twist_filter::param_callback, this);
    twist_filter::g_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    ros::spin();
    return 0;
}
