// Copyright (c) 2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** \brief This file defines the KartechLinearActuatorInterfaceCAN class.
 * \copyright Copyright 2021 New Eagle LLC
 * \file kartech_linear_actuator_interface_can.hpp
 */

#ifndef KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN__KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN_HPP_
#define KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN__KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN_HPP_

#include <cmath>
#include <array>
#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

// ROS messages
#include "can_msgs/msg/frame.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/brake_control.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/brake_position_report.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/kd_freq_deadband_request.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/kp_ki_request.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/pwm_frequency_request.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/heartbeat.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/left_joystick.hpp"
#include "kartech_linear_actuator_interface_msgs/msg/right_joystick.hpp"

#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"
#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"

#include "kartech_linear_actuator_interface_can/dispatch.hpp"

using can_msgs::msg::Frame;
using NewEagle::DbcMessage;

using kartech_linear_actuator_interface_msgs::msg::BrakeControl;
using kartech_linear_actuator_interface_msgs::msg::BrakePositionReport;
using kartech_linear_actuator_interface_msgs::msg::KdFreqDeadbandRequest;
using kartech_linear_actuator_interface_msgs::msg::KpKiRequest;
using kartech_linear_actuator_interface_msgs::msg::PwmFrequencyRequest;
using kartech_linear_actuator_interface_msgs::msg::Heartbeat;
using kartech_linear_actuator_interface_msgs::msg::LeftJoystick;
using kartech_linear_actuator_interface_msgs::msg::RightJoystick;

namespace kartech_linear_actuator_interface_can
{
class KartechLinearActuatorInterfaceCAN : public rclcpp::Node
{
public:
/** \brief Default constructor.
 * \param[in] options The options for this node.
 */
  explicit KartechLinearActuatorInterfaceCAN(const rclcpp::NodeOptions & options);

private:
/** \brief Convert reports received over CAN into ROS messages.
 * \param[in] msg The message received over CAN.
 */
  void recvCAN(const Frame::SharedPtr msg);

  void recvBrakePositionReport(const Frame::SharedPtr msg, DbcMessage * message);

  void recvBrakeControl(const BrakeControl::SharedPtr msg);

  void recvKdFreqDeadbandRequest(const KdFreqDeadbandRequest::SharedPtr msg);

  void recvKpKiRequest(const KpKiRequest::SharedPtr msg);
  void recvHeartbeat(const Frame::SharedPtr msg, DbcMessage * message);
  void recvLeftJoystick(const Frame::SharedPtr msg, DbcMessage * message);
  void recvRightJoystick(const Frame::SharedPtr msg, DbcMessage * message);

  void recvPwmFrequencyRequest(const PwmFrequencyRequest::SharedPtr msg);

  std::uint8_t vehicle_number_;

  // Parameters from launch
  std::string dbc_file_;
  float max_steer_angle_;
  bool publish_my_laps_;

  rclcpp::Subscription<BrakeControl>::SharedPtr subBrakeControl_;
  rclcpp::Subscription<BrakePositionReport>::SharedPtr subBrakePositionReport_;
  rclcpp::Subscription<KdFreqDeadbandRequest>::SharedPtr subKdFreqDeadbandRequest_;
  rclcpp::Subscription<KpKiRequest>::SharedPtr subKpKiRequest_;
  rclcpp::Subscription<PwmFrequencyRequest>::SharedPtr subPwmFrequencyRequest_;
  rclcpp::Subscription<Frame>::SharedPtr sub_can_;

  rclcpp::Publisher<BrakePositionReport>::SharedPtr pubBrakePositionReport_;
  rclcpp::Publisher<Heartbeat>::SharedPtr pubHeartbeat_;
  rclcpp::Publisher<LeftJoystick>::SharedPtr pubLeftJoystick_;
  rclcpp::Publisher<RightJoystick>::SharedPtr pubRightJoystick_;
  rclcpp::Publisher<Frame>::SharedPtr pub_can_;

  NewEagle::Dbc dbc_;
};

}  // namespace kartech_linear_actuator_interface_can

#endif  // KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN__KARTECH_LINEAR_ACTUATOR_INTERFACE_CAN_HPP_
