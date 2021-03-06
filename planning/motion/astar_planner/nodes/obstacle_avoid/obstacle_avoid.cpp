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

#include "obstacle_avoid.h"

using namespace obstacle_avoid;

aslan_msgs::Lane obstacle_avoid::createPublishWaypoints(const aslan_msgs::Lane& ref_lane, int closest_waypoint, int size)
{
  aslan_msgs::Lane follow_lane;

  follow_lane.header = ref_lane.header;
  follow_lane.increment = ref_lane.increment;

  // Push "size" waypoints from closest
  for (int i = 0; i < size; i++)
  {
    if (closest_waypoint + i >= static_cast<int>(ref_lane.waypoints.size()))
      break;

    follow_lane.waypoints.push_back(ref_lane.waypoints[closest_waypoint + i]);
  }

  return follow_lane;
}

void obstacle_avoid::createAvoidWaypoints(const nav_msgs::Path& astar_path, const astar_planner::SearchInfo& search_info, int size, aslan_msgs::Lane* avoid_lane, int* end_of_avoid_index)
{
  int closest_waypoint_index = search_info.getClosestWaypointIndex();

  avoid_lane->waypoints.clear();

  // Get global lane
  const aslan_msgs::Lane& current_lane = search_info.getCurrentWaypoints();
  avoid_lane->header = current_lane.header;
  avoid_lane->increment = current_lane.increment;

  // Set waypoints from closest to beginning of avoiding
  for (int i = closest_waypoint_index; i < search_info.getStartWaypointIndex(); i++)
  {
    avoid_lane->waypoints.push_back(current_lane.waypoints.at(i));
  }

  double avoid_velocity = avoid_lane->waypoints.back().twist.twist.linear.x;

  if (avoid_velocity > search_info.getAvoidVelocityLimitMPS())
    avoid_velocity = search_info.getAvoidVelocityLimitMPS();

  // Set waypoints for avoiding
  for (const auto& pose : astar_path.poses)
  {
    aslan_msgs::Waypoint wp;
    wp.pose = pose;
    wp.twist.twist.linear.x = avoid_velocity;

    avoid_lane->waypoints.push_back(wp);
  }

  // To know here is the end of avoiding
  *end_of_avoid_index = avoid_lane->waypoints.size();

  // Set waypoints from the end of avoiding
  for (int i = search_info.getGoalWaypointIndex() + 1; i < search_info.getGoalWaypointIndex() + size; i++)
  {
    if (i >= static_cast<int>(current_lane.waypoints.size()))
      break;

    avoid_lane->waypoints.push_back(current_lane.waypoints.at(i));
  }
}

int OBSTACLE_AVOID::RUN(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoid");
  ros::NodeHandle n;

  astar_planner::AstarSearch astar;
  astar_planner::SearchInfo search_info;

  // ROS subscribers
  ros::Subscriber map_sub = n.subscribe("grid_map_visualization/distance_transform", 1,
                                        &astar_planner::SearchInfo::mapCallback, &search_info);
  ros::Subscriber start_sub =
      n.subscribe("current_pose", 1, &astar_planner::SearchInfo::currentPoseCallback, &search_info);
  ros::Subscriber waypoints_sub =
      n.subscribe("base_waypoints", 1, &astar_planner::SearchInfo::waypointsCallback, &search_info);
  ros::Subscriber obstacle_waypoint_sub =
      n.subscribe("obstacle_waypoint", 1, &astar_planner::SearchInfo::obstacleWaypointCallback, &search_info);
  ros::Subscriber closest_waypoint_sub =
      n.subscribe("closest_waypoint", 1, &astar_planner::SearchInfo::closestWaypointCallback, &search_info);
  // TODO: optional
  // ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 1, &astar_planner::SearchInfo::goalCallback,
  // &search_info);
  ros::Subscriber current_velocity_sub =
      n.subscribe("current_velocity", 1, &astar_planner::SearchInfo::currentVelocityCallback, &search_info);
  ros::Subscriber state_sub = n.subscribe("state", 1, &astar_planner::SearchInfo::stateCallback, &search_info);

  // ROS publishers
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("astar_path", 1, true);
  ros::Publisher waypoints_pub = n.advertise<aslan_msgs::Lane>("safety_waypoints", 1, true);

  ros::Rate loop_rate(10);

  // variables for avoidance
  aslan_msgs::Lane avoid_lane;
  int end_of_avoid_index = -1;
  bool avoidance = false;
  while (ros::ok())
  {
    ros::spinOnce();

    int closest_waypoint;

    // We switch 2 waypoints, original path and avoiding path
    if (avoidance)
      closest_waypoint = getClosestWaypoint(avoid_lane, search_info.getCurrentPose().pose);
    else
      closest_waypoint = search_info.getClosestWaypointIndex();

    // there are no waypoints we can follow
    if (closest_waypoint < 0 || !search_info.getPathSet())
    {
      loop_rate.sleep();
      continue;
    }

    // Follow the original waypoints
    if (!avoidance)
    {
      aslan_msgs::Lane publish_lane;
      publish_lane = createPublishWaypoints(search_info.getSubscribedWaypoints(), closest_waypoint, 100);
      waypoints_pub.publish(publish_lane);
    }
    // Follow the avoiding waypoints
    else
    {
      // create waypoints from closest on avoid_lane
      aslan_msgs::Lane publish_lane;
      publish_lane = createPublishWaypoints(avoid_lane, closest_waypoint, 100);
      std::cout << "Avoid Lane: " << avoid_lane << std::endl;
      waypoints_pub.publish(publish_lane);

      // End of avoidance
      if (closest_waypoint > end_of_avoid_index)
      {
        avoidance = false;

        // Return to the original waypoints
        search_info.setCurrentWaypoints(search_info.getSubscribedWaypoints());

        loop_rate.sleep();
        continue;
      }
    }

    // Initialize vector for A* search, this runs only once
    if (search_info.getMapSet() && !astar.getNodeInitialized())
      astar.initializeNode(search_info.getMap());

    // Waiting for the call for avoidance ...
    if (!search_info.getMapSet() || !search_info.getStartSet() || !search_info.getGoalSet())
    {
      search_info.reset();
      loop_rate.sleep();
      continue;
    }

    // Run astar search
    ros::WallTime timer_begin = ros::WallTime::now();

    bool result = astar.makePlan(search_info.getStartPose().pose, search_info.getGoalPose().pose, search_info.getMap(),
                                 search_info.getUpperBoundDistance());

    ros::WallTime timer_end = ros::WallTime::now();
    double time_ms = (timer_end - timer_begin).toSec() * 1000;
    ROS_INFO("planning time: %lf [ms]", time_ms);

    // debug mode
    if (!search_info.getChangePath())
    {
      static double msec_sum = 0;
      static int plan_count = 0;
      plan_count++;
      msec_sum += time_ms;
      std::cout << "average time so far: " << msec_sum / plan_count << std::endl;
    }

    if (result)
    {
      std::cout << "Found goal!" << std::endl;
      path_pub.publish(astar.getPath());

      createAvoidWaypoints(astar.getPath(), search_info, 100, &avoid_lane, &end_of_avoid_index);

      if (search_info.getChangePath())
        avoidance = true;
    }
    else
    {
      std::cout << "can't find goal..." << std::endl;
    }

    // Reset flags
    search_info.reset();
    astar.reset();

    loop_rate.sleep();
  }

  return 0;
}
