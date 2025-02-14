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

#ifndef HIWIN_DRIVER_HARDWARE_INTERFACE_H_
#define HIWIN_DRIVER_HARDWARE_INTERFACE_H_

#include <vector>
#include <string>
#include <memory>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>

#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>

#include <std_srvs/Trigger.h>

#include <pass_through_controllers/joint_trajectory_interface.h>

#include <industrial_robot_status_interface/industrial_robot_status_interface.h>

#include <hiwin_robot_client_library/hiwin_driver.hpp>

namespace hiwin_driver
{

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  HardwareInterface();
  virtual ~HardwareInterface() = default;
  /*!
   * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns True, if the setup was performed successfully
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /*!
   * \brief Read method of the control loop. Reads data from the robot and handles and
   * publishes the information as needed.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Write method of the control loop. Writes target joint positions to the robot
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  /*!
   * \brief Starts and stops controllers.
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  /*!
   * \brief Checks if a reset of the ROS controllers is necessary.
   *
   * \returns Necessity of ROS controller reset
   */
  bool shouldResetControllers();

protected:
  void startJointInterpolation(const control_msgs::FollowJointTrajectoryGoal& trajectory);
  void abortMotion();
  bool clearErrorCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::JointTrajectoryInterface jnt_traj_interface_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;

  std::atomic<bool> controller_reset_necessary_;
  ros::ServiceServer srv_clear_error_;

  industrial_robot_status_interface::RobotStatus robot_status_resource_{};
  industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};

  std::string robot_ip_;
  std::string robot_version_;
  bool is_min_version_met;
  std::unique_ptr<hrsdk::HIWINDriver> hiwin_driver_;
};

}  // namespace hiwin_driver

#endif  // HIWIN_DRIVER_HARDWARE_INTERFACE_H_
