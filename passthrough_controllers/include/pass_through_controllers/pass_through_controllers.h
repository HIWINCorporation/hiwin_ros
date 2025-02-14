/*
 * -- BEGIN LICENSE BLOCK ----------------------------------------------
 * Copyright 2024 HIWIN Technologies Corp.
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
 *
 * -- END LICENSE BLOCK ------------------------------------------------
 */

#pragma once

#include <vector>
#include <memory>

#include <ros/ros.h>

#include <controller_interface/controller.h>

#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <realtime_tools/realtime_publisher.h>

#include "pass_through_controllers/joint_trajectory_interface.h"

namespace pass_through_controllers
{

class JointTrajectoryController : public controller_interface::Controller<hardware_interface::JointTrajectoryInterface>
{
public:
  bool init(hardware_interface::JointTrajectoryInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  void starting(const ros::Time& time);

  void stopping(const ros::Time& /*time*/);

  void update(const ros::Time& time, const ros::Duration& period);

  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);

private:
  hardware_interface::JointTrajectoryInterface* joint_trajectory_interface_;
  std::vector<hardware_interface::JointTrajectoryHandle> joints_;
  unsigned int num_hw_joints_;

  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::FollowJointTrajectoryFeedback>>
      pub_joint_control_state_;

  ros::Subscriber sub_joint_trajectory_;

  double state_publisher_rate_;
  bool pub_time_initialized_;
  ros::Time last_state_publish_time_;
};

}  // namespace pass_through_controllers