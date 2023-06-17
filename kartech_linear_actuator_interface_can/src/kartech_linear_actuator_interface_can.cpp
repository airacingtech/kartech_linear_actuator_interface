// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
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

#include <cmath>
#include <algorithm>
#include <string>

#include "kartech_linear_actuator_interface_can/kartech_linear_actuator_interface_can.hpp"

using std::chrono::duration;

namespace kartech_linear_actuator_interface_can
{

static constexpr uint64_t MS_IN_SEC = 1000;

KartechLinearActuatorInterfaceCAN::KartechLinearActuatorInterfaceCAN(
  const rclcpp::NodeOptions & options)
: Node("kartech_linear_actuator_interface_can_node", options)
{

  dbc_file_ = declare_parameter<std::string>("dbc_file", "");

  pub_can_ = this->create_publisher<Frame>(
    "can_rx", 20
  );
  pubBrakePositionReport_ = this->create_publisher<BrakePositionReport>(
    "brake_position_report",
    rclcpp::SensorDataQoS());
  pubHeartbeat_ = this->create_publisher<Heartbeat>("heartbeat", rclcpp::SensorDataQoS());
  pubLeftJoystick_ = this->create_publisher<LeftJoystick>("left_joystick", rclcpp::SensorDataQoS());
  pubRightJoystick_ = this->create_publisher<RightJoystick>(
    "right_joystick",
    rclcpp::SensorDataQoS());
  pubFrontRightWheelEncoder_ = this->create_publisher<FrontRightWheelEncoder>(
    "front_right_wheel_encoder", rclcpp::SensorDataQoS());
  pubFrontLeftWheelEncoder_ = this->create_publisher<FrontLeftWheelEncoder>(
    "front_left_wheel_encoder", rclcpp::SensorDataQoS());
  pubRearAxleWheelEncoder_ = this->create_publisher<RearAxleWheelEncoder>(
    "rear_axle_wheel_encoder",
    rclcpp::SensorDataQoS());

  subBrakeControl_ = this->create_subscription<BrakeControl>(
    "brake_control",
    rclcpp::SensorDataQoS(),
    std::bind(&KartechLinearActuatorInterfaceCAN::recvBrakeControl, this, std::placeholders::_1));
  subKdFreqDeadbandRequest_ = this->create_subscription<KdFreqDeadbandRequest>(
    "kd_freq_deadband_request", rclcpp::SensorDataQoS(),
    std::bind(
      &KartechLinearActuatorInterfaceCAN::recvKdFreqDeadbandRequest, this,
      std::placeholders::_1));
  subKpKiRequest_ = this->create_subscription<KpKiRequest>(
    "kp_ki_request",
    rclcpp::SensorDataQoS(),
    std::bind(&KartechLinearActuatorInterfaceCAN::recvKpKiRequest, this, std::placeholders::_1));
  subPwmFrequencyRequest_ = this->create_subscription<PwmFrequencyRequest>(
    "pwm_frequency_request",
    rclcpp::SensorDataQoS(),
    std::bind(
      &KartechLinearActuatorInterfaceCAN::recvPwmFrequencyRequest, this,
      std::placeholders::_1));
  sub_can_ = this->create_subscription<Frame>(
    "can_tx", 500,
    std::bind(&KartechLinearActuatorInterfaceCAN::recvCAN, this, std::placeholders::_1)
  );

  dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);
}

#define RECV_DBC(handler) \
  message = dbc_.GetMessageById(id); \
  if (msg->dlc >= message->GetDlc()) {message->SetFrame(msg); handler(msg, message);}

void KartechLinearActuatorInterfaceCAN::recvCAN(const Frame::SharedPtr msg)
{
  NewEagle::DbcMessage * message = nullptr;
  if (!msg->is_rtr && !msg->is_error) {
    auto id = msg->id;
    switch (id) {
      case ID_BRAKE_POSITION_REPORT:
        RECV_DBC(recvBrakePositionReport);
        break;
      case ID_HEARTBEAT:
        RECV_DBC(recvHeartbeat);
        break;
      case ID_LEFT_JOYSTICK:
        RECV_DBC(recvLeftJoystick);
        break;
      case ID_RIGHT_JOYSTICK:
        RECV_DBC(recvRightJoystick);
        break;
      case ID_FRONT_RIGHT_WHEEL_ENCODER:
        RECV_DBC(recvFrontRightWheelEncoder);
        break;
      case ID_FRONT_LEFT_WHEEL_ENCODER:
        RECV_DBC(recvFrontLeftWheelEncoder);
        break;
      case ID_REAR_AXLE_WHEEL_ENCODER:
        RECV_DBC(recvRearAxleWheelEncoder);
        break;
      default:
        break;
    }
  }
}

void KartechLinearActuatorInterfaceCAN::recvBrakePositionReport(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  BrakePositionReport out;
  out.stamp = msg->header.stamp;

  out.messagetype = message->GetSignal("MessageType")->GetResult();
  out.confirmationflag = message->GetSignal("ConfirmationFlag")->GetResult();
  out.autoreplyflag = message->GetSignal("AutoReplyFlag")->GetResult();
  out.datatype = message->GetSignal("DataType")->GetResult();
  out.shaftextension = message->GetSignal("ShaftExtension")->GetResult();

  pubBrakePositionReport_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvBrakeControl(const BrakeControl::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbc_.GetMessage("Brake_Control");

  message->GetSignal("Position_Command")->SetResult(msg->position_command);
  message->GetSignal("Datatype")->SetResult(msg->datatype);
  message->GetSignal("Autoreply_Flag")->SetResult(msg->autoreply_flag);
  message->GetSignal("Confirmation_Flag")->SetResult(msg->confirmation_flag);
  message->GetSignal("DPOS_LOW")->SetResult(msg->dpos_low);
  message->GetSignal("DPOS_HI")->SetResult(msg->dpos_hi);
  message->GetSignal("Motor_Enable")->SetResult(msg->motor_enable);
  message->GetSignal("Clutch_Enable")->SetResult(msg->clutch_enable);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void KartechLinearActuatorInterfaceCAN::recvKdFreqDeadbandRequest(
  const KdFreqDeadbandRequest::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbc_.GetMessage("KD_Freq_Deadband_Request");

  message->GetSignal("MessageType")->SetResult(msg->messagetype);
  message->GetSignal("DataType")->SetResult(msg->datatype);
  message->GetSignal("AutoReplyFlag")->SetResult(msg->autoreplyflag);
  message->GetSignal("ConfirmationFlag")->SetResult(msg->confirmationflag);
  message->GetSignal("Byte2")->SetResult(msg->byte2);
  message->GetSignal("Byte3")->SetResult(msg->byte3);
  message->GetSignal("KD")->SetResult(msg->kd);
  message->GetSignal("ErrorDeadBand")->SetResult(msg->errordeadband);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void KartechLinearActuatorInterfaceCAN::recvKpKiRequest(const KpKiRequest::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbc_.GetMessage("KP_KI_Request");

  message->GetSignal("MessageType")->SetResult(msg->messagetype);
  message->GetSignal("DataType")->SetResult(msg->datatype);
  message->GetSignal("AutoReplyFlag")->SetResult(msg->autoreplyflag);
  message->GetSignal("ConfirmationFlag")->SetResult(msg->confirmationflag);
  message->GetSignal("Byte2")->SetResult(msg->byte2);
  message->GetSignal("Byte3")->SetResult(msg->byte3);
  message->GetSignal("KP")->SetResult(msg->kp);
  message->GetSignal("KI")->SetResult(msg->ki);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void KartechLinearActuatorInterfaceCAN::recvPwmFrequencyRequest(
  const PwmFrequencyRequest::SharedPtr msg)
{
  NewEagle::DbcMessage * message = dbc_.GetMessage("PWM_Frequency_Request");

  message->GetSignal("MessageType")->SetResult(msg->messagetype);
  message->GetSignal("DataType")->SetResult(msg->datatype);
  message->GetSignal("AutoReplyFlag")->SetResult(msg->autoreplyflag);
  message->GetSignal("ConfirmationFlag")->SetResult(msg->confirmationflag);
  message->GetSignal("Byte2")->SetResult(msg->byte2);
  message->GetSignal("Byte3")->SetResult(msg->byte3);
  message->GetSignal("PWM_MIN")->SetResult(msg->pwm_min);
  message->GetSignal("PWM_MAX")->SetResult(msg->pwm_max);
  message->GetSignal("PWM_FREQ")->SetResult(msg->pwm_freq);

  Frame frame = message->GetFrame();
  pub_can_->publish(frame);
}

void KartechLinearActuatorInterfaceCAN::recvHeartbeat(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  Heartbeat out;
  out.stamp = msg->header.stamp;

  out.vehicle_safety_controller_mode =
    message->GetSignal("Vehicle_Safety_Controller_Mode")->GetResult();
  out.autonomy_mode = message->GetSignal("Autonomy_Mode")->GetResult();
  out.e_stop_indication = message->GetSignal("E_Stop_Indication")->GetResult();

  pubHeartbeat_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvLeftJoystick(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  LeftJoystick out;
  out.stamp = msg->header.stamp;

  out.left_joystick_x = message->GetSignal("Left_Joystick_X")->GetResult();
  out.left_joystick_y = message->GetSignal("Left_Joystick_Y")->GetResult();
  out.left_button = message->GetSignal("Left_Button")->GetResult();

  pubLeftJoystick_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvRightJoystick(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  RightJoystick out;
  out.stamp = msg->header.stamp;

  out.right_joystick_x = message->GetSignal("Right_Joystick_X")->GetResult();
  out.right_joystick_y = message->GetSignal("Right_Joystick_Y")->GetResult();
  out.right_button = message->GetSignal("Right_Button")->GetResult();

  pubRightJoystick_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvFrontRightWheelEncoder(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  FrontRightWheelEncoder out;
  out.stamp = msg->header.stamp;

  out.velocity = message->GetSignal("Velocity")->GetResult();

  pubFrontRightWheelEncoder_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvFrontLeftWheelEncoder(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  FrontLeftWheelEncoder out;
  out.stamp = msg->header.stamp;

  out.velocity = message->GetSignal("Velocity")->GetResult();

  pubFrontLeftWheelEncoder_->publish(out);
}

void KartechLinearActuatorInterfaceCAN::recvRearAxleWheelEncoder(
  const Frame::SharedPtr msg,
  DbcMessage * message)
{
  RearAxleWheelEncoder out;
  out.stamp = msg->header.stamp;

  out.velocity = message->GetSignal("Velocity")->GetResult();

  pubRearAxleWheelEncoder_->publish(out);
}

}  // namespace kartech_linear_actuator_interface_can
