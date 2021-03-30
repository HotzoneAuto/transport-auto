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

#include "modules/canbus/protocol/id_0x184.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

Id0x184::Id0x184() {}
const int32_t Id0x184::ID = 0x184;

void Id0x184::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  // TODO: update chassis_detail.proto
  chassis->set_req_transport_come_flag(req_transport_come_flag(bytes, length));
  chassis->set_digger_load_complete_flag(digger_load_complete_flag(bytes, length));
  chassis->set_req_transport_emergency_stop_flag(req_transport_emergency_stop_flag(bytes, length));
}

int Id0x184::req_transport_come_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 2);

  int ret = x;
  return ret;
}

int Id0x184::digger_load_complete_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(2, 4);

  int ret = x;
  return ret;
}

int Id0x184::req_transport_emergency_stop_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
