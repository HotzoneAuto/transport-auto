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
#include "modules/canbus/protocol/id_0x301.h"
#include "modules/canbus/protocol/id_0x302.h"
#include "modules/canbus/protocol/id_0x303.h"
#include "modules/canbus/protocol/id_0x304.h"
#include "modules/canbus/protocol/id_0x1314.h"
#include "modules/canbus/protocol/id_0x1315.h"

namespace apollo {
namespace canbus {
namespace transport {

TransportMessageManager::TransportMessageManager() {
  // Control Messages
  AddSendProtocolData<Id0xc040b2b, true>();
  AddSendProtocolData<Id0x4ef8480, true>();
  AddSendProtocolData<Id0x302, true>();
  AddSendProtocolData<Id0x1314, true>();

  // Report Messages
  AddRecvProtocolData<Id0x18ff4bd1, true>();
  AddRecvProtocolData<Id0x301, true>();
  AddRecvProtocolData<Id0x303, true>();
  AddRecvProtocolData<Id0x304, true>();
  AddRecvProtocolData<Id0x1315, true>();
}

TransportMessageManager::~TransportMessageManager() {}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
