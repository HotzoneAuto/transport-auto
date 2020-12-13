/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include "modules/common/proto/error_code.pb.h"

#include "modules/transport_can/proto/control_command.pb.h"
#include "modules/transport_can/vehicle/transport/protocol/id_0x4ef8480.h"
#include "modules/transport_can/vehicle/transport/protocol/id_0xc040b2b.h"

#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/transport_can/vehicle/transport/transport_message_manager.h"
using ::apollo::canbus::ControlCommand;
using ::apollo::canbus::Transport::Id0x4ef8480;
using ::apollo::canbus::Transport::Id0xc040b2b;
using ::apollo::common::ErrorCode;
using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::ProtocolData;

class TransportController {
 public:
  explicit TransportController(){};

  virtual ~TransportController();

  ::apollo::common::ErrorCode Init(
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender_,
      MessageManager<::apollo::canbus::ChassisDetail> *const message_manager);
  void ControlUpdate(ControlCommand cmd, const int SteerEnable,
                     const int AccEnable);
  void Start();
  void Stop();

 private:
  bool is_initialized_ = false;
  bool is_start = false;
  // control protocol
  CanSender<::apollo::canbus::ChassisDetail> *can_sender_;
  MessageManager<::apollo::canbus::ChassisDetail> *message_manager_;
  Id0x4ef8480 *id_0x4ef8480_ = nullptr;
  Id0xc040b2b *id_0xc040b2b_ = nullptr;
};
