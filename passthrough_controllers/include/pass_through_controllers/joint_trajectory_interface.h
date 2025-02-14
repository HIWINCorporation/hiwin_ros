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

#include <functional>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{

class JointTrajectoryHandle : public JointStateHandle
{
public:
  JointTrajectoryHandle() = default;

  JointTrajectoryHandle(const JointStateHandle& js) : JointStateHandle(js)
  {
  }
};

class JointTrajectoryInterface : public HardwareResourceManager<JointTrajectoryHandle, ClaimResources>
{
public:
  void registerGoalCallback(std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> f)
  {
    goal_callback_ = f;
  }

  bool setGoal(control_msgs::FollowJointTrajectoryGoal goal)
  {
    if (goal_callback_ != nullptr)
    {
      goal_callback_(goal);
      return true;
    }
    return false;
  }

  void registerCancelCallback(std::function<void()> f)
  {
    cancel_callback_ = f;
  }

  void setCancel()
  {
    if (cancel_callback_ != nullptr)
      cancel_callback_();
  }

  void setFeedback(control_msgs::FollowJointTrajectoryFeedback feedback)
  {
    feedback_ = feedback;
  }

  control_msgs::FollowJointTrajectoryFeedback getFeedback() const
  {
    return feedback_;
  }

private:
  std::function<void(const control_msgs::FollowJointTrajectoryGoal&)> goal_callback_;
  std::function<void()> cancel_callback_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
};

}  // namespace hardware_interface