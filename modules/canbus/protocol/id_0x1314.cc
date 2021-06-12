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

#include "modules/canbus/protocol/id_0x1314.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x1314::ID = 0x1314;

// public
Id0x1314::Id0x1314() { Reset(); }

uint32_t Id0x1314::GetPeriod() const {
  static const uint32_t PERIOD = 200 * 1000;
  return PERIOD;
}

void Id0x1314::UpdateData(uint8_t* data) {
  set_p_gearshiftcmd(data, gearshiftcmd_);
}

void Id0x1314::Reset() {
  gearshiftcmd_ = 0;
}

Id0x1314* Id0x1314::set_gearshiftcmd(int gearshiftcmd) {
  gearshiftcmd_ = gearshiftcmd;
  return this;
}

void Id0x1314::set_p_gearshiftcmd(uint8_t* data, int gearshiftcmd) {
  gearshiftcmd = ProtocolData::BoundedValue(0, 6, gearshiftcmd);
  int x = gearshiftcmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
