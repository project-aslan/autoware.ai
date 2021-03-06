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

#include "waypoint_follower/pure_pursuit_core.h"

using namespace std;

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , is_velocity_set_(false)
  , is_config_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(0)
  , param_flag_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(11)
  , lookahead_distance_ratio_(2.0)
  , minimum_lookahead_distance_(6.0)
{
  initForROS();

  // initialize for PurePursuit
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
  // ROS_INFO_STREAM("is_linear_interpolation : " << is_linear_interpolation_);
  private_nh_.param("publishes_for_steering_robot", publishes_for_steering_robot_, bool(false));
  nh_.param("vehicle_info/wheel_base", wheel_base_, double(1.68));

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 100, &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
  sub3_ = nh_.subscribe("config/waypoint_follower", 10, &PurePursuitNode::callbackFromConfig, this);
  sub4_ = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub2_ = nh_.advertise<aslan_msgs::ControlCommandStamped>("ctrl_cmd", 10);
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
  // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_)
    {
      ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());
    pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);
    publishTwistStamped(can_get_curvature, kappa);
    publishControlCommandStamped(can_get_curvature, kappa);

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(pp_.getCurrentPose().position, pp_.getCurrentWaypoints());

    is_pose_set_ = false;
    is_velocity_set_ = false;
    is_waypoint_set_ = false;

    loop_rate.sleep();
  }
}

void PurePursuitNode::publishTwistStamped(const bool &can_get_curvature, const double &kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(ts);
}

void PurePursuitNode::publishControlCommandStamped(const bool &can_get_curvature, const double &kappa) const
{
  if (!publishes_for_steering_robot_)
    return;

  aslan_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;

  pub2_.publish(ccs);
}

double PurePursuitNode::computeLookaheadDistance() const
{
  if (param_flag_ == enumToInteger(Mode::dialog) || param_flag_ == -1)
    return const_lookahead_distance_;

  double maximum_lookahead_distance = current_linear_velocity_ * 10;
  double ld = current_linear_velocity_ * lookahead_distance_ratio_;

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
                                            ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

double PurePursuitNode::computeLookaheadDistance_test()
{
  return PurePursuitNode::computeLookaheadDistance();
}

void PurePursuitNode::set_param_flag_(int32_t param_flag)
{
    PurePursuitNode::param_flag_ = param_flag; 
}

void PurePursuitNode::set_minimum_lookahead_distance_(double minimum_lookahead_distance)
{
    PurePursuitNode::minimum_lookahead_distance_ = minimum_lookahead_distance; 
}

void PurePursuitNode::set_const_lookahead_distance_(double const_lookahead_distance)
{
    PurePursuitNode::const_lookahead_distance_ = const_lookahead_distance; 
}

void PurePursuitNode::set_current_linear_velocity_(double current_linear_velocity)
{
    PurePursuitNode::current_linear_velocity_ = current_linear_velocity; 
}

void PurePursuitNode::set_lookahead_distance_ratio_(double lookahead_distance_ratio)
{
    PurePursuitNode::lookahead_distance_ratio_ = lookahead_distance_ratio;
}

double PurePursuitNode::computeCommandVelocity() const
{
    if (param_flag_ == enumToInteger(Mode::dialog))
    {
        if (command_linear_velocity_ == 0.0)
            return command_linear_velocity_;
        return kmph2mps(const_velocity_);
    }
    else if (param_flag_ == -1)
      return 0;
    return command_linear_velocity_;
}

void PurePursuitNode::set_command_linear_velocity_(double command_linear_velocity)
{
  PurePursuitNode::command_linear_velocity_ = command_linear_velocity;
}

double PurePursuitNode::computeCommandVelocity_test()
{
  return PurePursuitNode::computeCommandVelocity();
}

double PurePursuitNode::computeCommandAccel() const
{
  const geometry_msgs::Pose current_pose = pp_.getCurrentPose();
  const geometry_msgs::Pose target_pose = pp_.getCurrentWaypoints().at(1).pose.pose;

  // v^2 - v0^2 = 2ax
  const double x =
      std::hypot(current_pose.position.x - target_pose.position.x, current_pose.position.y - target_pose.position.y);
  const double v0 = current_linear_velocity_;
  const double v = computeCommandVelocity();
  const double a = (v * v - v0 * v0) / (2 * x);
    return a;
}

double PurePursuitNode::computeCommandAccel_test()
{
  return PurePursuitNode::computeCommandAccel();
}

double PurePursuitNode::computeAngularGravity(double velocity, double kappa) const
{
  const double gravity = 9.80665;
  return (velocity * velocity) / (1.0 / kappa * gravity);
}

double PurePursuitNode::computeAngularGravity_test(double velocity, double kappa)
{
  return PurePursuitNode::computeAngularGravity(velocity, kappa);
}

void PurePursuitNode::callbackFromConfig(const aslan_msgs::ConfigWaypointFollowerConstPtr &config)
{
  param_flag_ = config->param_flag;
  const_lookahead_distance_ = config->lookahead_distance;
  const_velocity_ = config->velocity;
  lookahead_distance_ratio_ = config->lookahead_ratio;
  minimum_lookahead_distance_ = config->minimum_lookahead_distance;
  is_config_set_ = true;
}

void PurePursuitNode::publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                                      const std::vector<aslan_msgs::Waypoint> &waypoints) const
{
  // Calculate the deviation of current position from the waypoint approximate line

  if (waypoints.size() < 3)
  {
    return;
  }

  double a, b, c;
  double linear_flag_in = getLinearEquation(waypoints.at(2).pose.pose.position, waypoints.at(1).pose.pose.position, &a, &b, &c);
  ROS_DEBUG("pure_pursuit linear_flag = %f", linear_flag_in);

  std_msgs::Float32 msg;
  msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);

  pub17_.publish(msg);
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_linear_velocity_ = msg->twist.linear.x;
  pp_.setCurrentVelocity(current_linear_velocity_);
  is_velocity_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const aslan_msgs::LaneConstPtr &msg)
{
  if (!msg->waypoints.empty()) {
      command_linear_velocity_ = msg->waypoints.at(0).twist.twist.linear.x;
      if (command_linear_velocity_ > 9){
        command_linear_velocity_ = 9;
      }
  }else{
    command_linear_velocity_ = 0;}

  pp_.setCurrentWaypoints(msg->waypoints);
  is_waypoint_set_ = true;
}

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
  return atan(wheel_base * kappa);
}

}  // waypoint_follower
