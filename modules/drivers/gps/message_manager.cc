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

#include "modules/drivers/gps/message_manager.h"

#include "modules/drivers/gps/protocol/id_0x701.h"
#include "modules/drivers/gps/protocol/id_0x703.h"
#include "modules/drivers/gps/protocol/id_0x710.h"
#include "modules/drivers/gps/protocol/id_0x712.h"
#include "modules/drivers/gps/protocol/id_0x713.h"

namespace apollo {
namespace drivers {
namespace gps {

TransportGPSMessageManager::TransportGPSMessageManager() {
  // Control Messages

  // Report Messages
  AddRecvProtocolData<protocol::Id0x701, true>();
  AddRecvProtocolData<protocol::Id0x703, true>();
  AddRecvProtocolData<protocol::Id0x710, true>();
  AddRecvProtocolData<protocol::Id0x712, true>();
  AddRecvProtocolData<protocol::Id0x713, true>();
}

TransportGPSMessageManager::~TransportGPSMessageManager() {}

}  // namespace gps
}  // namespace drivers
}  // namespace apollo
