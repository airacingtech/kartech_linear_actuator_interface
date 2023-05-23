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

KartechLinearActuatorInterfaceCAN::KartechLinearActuatorInterfaceCAN(const rclcpp::NodeOptions & options)
: Node("kartech_linear_actuator_interface_can_node", options)
{

    dbc_file_ = declare_parameter<std::string>("dbc_file", "");

    pub_can_ = this->create_publisher<Frame>(
        "can_rx", 20
    );
    pubBrakePositionReport_ = this->create_publisher<BrakePositionReport>("brake_position_report", rclcpp::SensorDataQoS());

    subBrakeControl_ = this->create_subscription<BrakeControl>("brake_control", rclcpp::SensorDataQoS(), std::bind(&KartechLinearActuatorInterfaceCAN::recvBrakeControl, this, std::placeholders::_1));
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
            default:
                break;
        }
    }
}

void KartechLinearActuatorInterfaceCAN::recvBrakePositionReport(const Frame::SharedPtr msg, DbcMessage * message)
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
	NewEagle::DbcMessage * message = dbc_.GetMessageById(ID_BRAKE_CONTROL);

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

}  // namespace kartech_linear_actuator_interface_can

