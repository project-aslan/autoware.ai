/*
 * Copyright (C) 2020 Project ASLAN - All rights reserved
 *
 * Author: Abdelrahman Barghouth
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include "map_tf_generator.h"

void map_tf_generator::Callback(const PointCloud::ConstPtr &clouds) {
  const unsigned int sum = clouds->points.size();
  double coordinate[3] = {0, 0, 0};
  for (int i = 0; i < sum; i++) {
    coordinate[0] += clouds->points[i].x;
    coordinate[1] += clouds->points[i].y;
    coordinate[2] += clouds->points[i].z;
  }
  coordinate[0] = -1 * coordinate[0] / sum;
  coordinate[1] = -1 * coordinate[1] / sum;
  coordinate[2] = -1 * coordinate[2] / sum;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(coordinate[0], coordinate[1], coordinate[2]));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    loop_rate.sleep();
  }
}

int map_tf_generator::RUN(int argc, char **argv) {
  ros::init(argc, argv, "map_tf_generator");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe<PointCloud>("/points_map", 1, &map_tf_generator::Callback, this);
  ros::spin();

  return 0;
};
