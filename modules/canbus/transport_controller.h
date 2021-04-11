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

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "gflags/gflags.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/canbus/proto/transport_can_conf.pb.h"
#include "modules/canbus/protocol/id_0x4ef8480.h"
#include "modules/canbus/protocol/id_0xc040b2b.h"
#include "modules/canbus/protocol/id_0x302.h"
#include "modules/canbus/protocol/id_0x1314.h"
#include "modules/canbus/protocol/id_0x2273.h"
#include "modules/canbus/transport_message_manager.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/common/time/time.h"
#include "modules/control/proto/control_command.pb.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {

using ::apollo::canbus::transport::Id0x4ef8480;
using ::apollo::canbus::transport::Id0xc040b2b;
using ::apollo::canbus::transport::Id0x302;
using ::apollo::canbus::transport::Id0x1314;
using ::apollo::canbus::transport::Id0x2273;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;
using ::apollo::drivers::canbus::ProtocolData;

class TransportController {
 public:
  explicit TransportController(){};

  virtual ~TransportController();

  ::apollo::common::ErrorCode Init(
      CanSender<::apollo::canbus::ChassisDetail> *const can_sender_,
      MessageManager<::apollo::canbus::ChassisDetail> *const message_manager);
  void ControlUpdate(ControlCommand cmd, const int SteerEnable,
                     const int AccEnable, float vol_cur, float vol_exp);
  void Start();
  void Stop();

 private:
  bool is_initialized_ = false;
  bool is_start = false;
  int count_flag = 0;

  // control protocol
  CanSender<::apollo::canbus::ChassisDetail> *can_sender_;
  MessageManager<::apollo::canbus::ChassisDetail> *message_manager_;
  apollo::canbus::TransportCanConf transport_can_conf_;
  Id0x4ef8480 *id_0x4ef8480_ = nullptr;
  Id0xc040b2b *id_0xc040b2b_ = nullptr;
  Id0x302 *id_0x302_ = nullptr;
  Id0x1314 *id_0x1314_ = nullptr;
  Id0x2273 *id_0x2273_ = nullptr;
};
}  // namespace canbus
}  // namespace apollo
