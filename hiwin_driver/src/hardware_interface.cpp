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

#include <std_srvs/Trigger.h>

#include "hiwin_driver/hardware_interface.h"

#define ROBOT_REQUIRED_VERSION "4.0.0"

using industrial_robot_status_interface::RobotMode;
using industrial_robot_status_interface::TriState;

namespace hiwin_driver
{
HardwareInterface::HardwareInterface()
{
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("robot_ip") << " not given.");
    return false;
  }

  // Names of the joints. Usually, this is given in the controller config file.
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                       << " on the parameter server.");
    return false;
  }

  joint_positions_.resize(joint_names_.size(), 0);
  joint_velocities_.resize(joint_names_.size(), 0);
  joint_efforts_.resize(joint_names_.size(), 0);

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint trajectory interface
    jnt_traj_interface_.registerHandle(
        hardware_interface::JointTrajectoryHandle(js_interface_.getHandle(joint_names_[i])));
  }

  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::abortMotion, this));

  robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
      "industrial_robot_status_handle", robot_status_resource_));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&jnt_traj_interface_);
  registerInterface(&robot_status_interface_);

  //
  hiwin_driver_.reset(new hrsdk::HIWINDriver(robot_ip_));
  hiwin_driver_->connect();
  hiwin_driver_->getRobotVersion(robot_version_);
  if (hiwin_driver_->isVersionGreaterOrEqual(ROBOT_REQUIRED_VERSION))
  {
    is_min_version_met = true;
  }
  else
  {
    is_min_version_met = false;
    ROS_WARN("Warning: Robot version (%s) is lower than the required version (%s). Please update the software.",
             robot_version_.c_str(), ROBOT_REQUIRED_VERSION);
  }

  srv_clear_error_ = root_nh.advertiseService("clear_error", &HardwareInterface::clearErrorCb, this);

  return true;
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  hrsdk::ControlMode ctrl_mode;
  hiwin_driver_->getRobotMode(ctrl_mode);
  if (ctrl_mode == hrsdk::ControlMode::Manual)
  {
    robot_status_resource_.mode = RobotMode::MANUAL;
  }
  else if (ctrl_mode == hrsdk::ControlMode::Auto)
  {
    robot_status_resource_.mode = RobotMode::AUTO;
  }
  else
  {
    robot_status_resource_.mode = RobotMode::UNKNOWN;
  }

  robot_status_resource_.drives_powered = (hiwin_driver_->isDrivesPowered()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.in_error = (hiwin_driver_->isInError()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.motion_possible = (hiwin_driver_->isMotionPossible()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.in_motion = (hiwin_driver_->isInMotion()) ? TriState::TRUE : TriState::FALSE;
  robot_status_resource_.e_stopped = (hiwin_driver_->isEstopped()) ? TriState::TRUE : TriState::FALSE;
  hiwin_driver_->getErrorCode(robot_status_resource_.error_code);

  hiwin_driver_->getJointPosition(joint_positions_);
  hiwin_driver_->getJointVelocity(joint_velocities_);

  control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    feedback.joint_names.push_back(joint_names_[i]);
    feedback.actual.positions.push_back(joint_positions_[i]);
    feedback.actual.velocities.push_back(joint_velocities_[i]);
  }
  jnt_traj_interface_.setFeedback(feedback);
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
}

void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (auto& controller_it : stop_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
    }
  }

  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
    }
  }
}

bool HardwareInterface::shouldResetControllers()
{
  if (controller_reset_necessary_)
  {
    controller_reset_necessary_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void HardwareInterface::startJointInterpolation(const control_msgs::FollowJointTrajectoryGoal& trajectory)
{
  size_t point_number = trajectory.trajectory.points.size();

  if (is_min_version_met)
  {
    double last_time = 0.0;

    for (size_t i = 0; i < point_number; i++)
    {
      trajectory_msgs::JointTrajectoryPoint point = trajectory.trajectory.points[i];
      std::vector<double> p;
      for (size_t j = 0; j < point.positions.size(); j++)
      {
        p.push_back(point.positions[j]);
      }

      double next_time = point.time_from_start.toSec();

      if (point.velocities.size() == point.positions.size() && point.accelerations.size() == point.positions.size())
      {
        std::vector<double> v;
        std::vector<double> a;
        for (size_t j = 0; j < point.positions.size(); j++)
        {
          v.push_back(point.velocities[j]);
          a.push_back(point.accelerations[j]);
        }

        hiwin_driver_->writeTrajectorySplinePoint(p, v, a, next_time - last_time);
      }
      else if (point.velocities.size() == point.positions.size())
      {
        std::vector<double> v;
        for (size_t j = 0; j < point.positions.size(); j++)
        {
          v.push_back(point.velocities[j]);
        }
        hiwin_driver_->writeTrajectorySplinePoint(p, v, next_time - last_time);
      }
      else
      {
        hiwin_driver_->writeTrajectorySplinePoint(p, next_time - last_time);
      }

      last_time = next_time;
    }
  }
  else
  {
    ROS_WARN("Warning: Robot version (%s) is lower than the required version (%s). Please update the software.",
             robot_version_.c_str(), ROBOT_REQUIRED_VERSION);

    for (size_t i = 0; i < point_number; i++)
    {
      trajectory_msgs::JointTrajectoryPoint point = trajectory.trajectory.points[i];
      std::vector<double> p;
      for (size_t j = 0; j < point.positions.size(); j++)
      {
        p.push_back(point.positions[j]);
      }
      hiwin_driver_->writeJointCommand(p);
    }
  }
}

void HardwareInterface::abortMotion()
{
  ROS_DEBUG("Abort motion");
  hiwin_driver_->motionAbort();
}

bool HardwareInterface::clearErrorCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.success = true;
  resp.message = "";

  hiwin_driver_->clearError();

  return true;
}

}  // namespace hiwin_driver

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hiwin_driver::HardwareInterface, hardware_interface::RobotHW);
