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

// ROS Includes
#include <ros/ros.h>

// User defined includes
#include "waypoint_follower/pure_pursuit_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  waypoint_follower::PurePursuitNode ppn(nh, private_nh);
  ppn.run();

  return 0;
}
