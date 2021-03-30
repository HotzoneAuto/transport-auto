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

#include "modules/canbus/transport_message_manager.h"

#include "modules/canbus/protocol/id_0x18ff4bd1.h"
#include "modules/canbus/protocol/id_0x4ef8480.h"
#include "modules/canbus/protocol/id_0xc040b2b.h"
#include "modules/canbus/protocol/id_0x184.h"
#include "modules/canbus/protocol/id_0x284.h"
#include "modules/canbus/protocol/id_0x384.h"
#include "modules/canbus/protocol/id_0x484.h"

namespace apollo {
namespace canbus {
namespace transport {

TransportMessageManager::TransportMessageManager() {
  // Control Messages
  AddSendProtocolData<Id0xc040b2b, true>();
  AddSendProtocolData<Id0x4ef8480, true>();
  AddSendProtocolData<Id0x284, true>();

  // Report Messages
  AddRecvProtocolData<Id0x18ff4bd1, true>();
  AddRecvProtocolData<Id0x184, true>();
  AddRecvProtocolData<Id0x384, true>();
  AddRecvProtocolData<Id0x484, true>();
}

TransportMessageManager::~TransportMessageManager() {}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
