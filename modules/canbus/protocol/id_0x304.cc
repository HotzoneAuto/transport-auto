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

#include "modules/canbus/protocol/id_0x304.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

Id0x304::Id0x304() {}
const int32_t Id0x304::ID = 0x304;

void Id0x304::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  // TODO: update chassis_detail.proto
  chassis->set_digger_altitude(digger_altitude(bytes, length));
  chassis->set_digger_longitude(digger_longitude(bytes, length));
}

double Id0x304::digger_altitude(const std::uint8_t* bytes, const int32_t length) const {
  Byte t1(bytes + 7);
  uint32_t x1 = t1.get_byte(0, 8);

  Byte t2(bytes + 6);
  uint32_t x2 = t2.get_byte(0, 8);

  Byte t3(bytes + 5);
  uint32_t x3 = t3.get_byte(0, 8);

  x1 <<= 16;
  x2 <<= 8;

  x1 |= x2;
  x1 |= x3;

  double ret = x1 * 0.001;
  return ret;
}

double Id0x304::digger_longitude(const std::uint8_t* bytes, const int32_t length) const {
  Byte t1(bytes + 4);
  int64_t x1 = t1.get_byte(0, 8);

  Byte t2(bytes + 3);
  int64_t x2 = t2.get_byte(0, 8);

  Byte t3(bytes + 2);
  int64_t x3 = t3.get_byte(0, 8);

  Byte t4(bytes + 1);
  int64_t x4 = t4.get_byte(0, 8);

  Byte t5(bytes + 0);
  int64_t x5 = t5.get_byte(0, 8);

  x1 <<= 32;
  x2 <<= 24;
  x3 <<= 16;
  x4 <<= 8;

  x1 |= x2;
  x1 |= x3;
  x1 |= x4;
  x1 |= x5;

  x1 <<= 24;
  x1 >>= 24;

  double ret = x1 * 1e-9;
  return ret;
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
