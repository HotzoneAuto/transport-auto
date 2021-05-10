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

#include "modules/canbus/protocol/id_0x100.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

Id0x100::Id0x100() {}
const int32_t Id0x100::ID = 0x100;

void Id0x100::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_stopfrmaster_flag(stopfrmaster_flag(bytes, length));
  chassis->set_missionfrmaster_flag(missionfrmaster_flag(bytes, length));
  chassis->set_trajfrmaster_flag(trajfrmaster_flag(bytes, length));
}

int Id0x100::stopfrmaster_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Id0x100::missionfrmaster_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Id0x100::trajfrmaster_flag(const std::uint8_t* bytes, const int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
