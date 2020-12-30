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

#include "modules/drivers/gps/protocol/id_0x713.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace drivers {
namespace gps {
namespace protocol {

using ::apollo::drivers::canbus::Byte;

Id0x713::Id0x713() {}
const int32_t Id0x713::ID = 0x713;

void Id0x713::Parse(const std::uint8_t* bytes, int32_t length,
                    apollo::drivers::Gps* chassis) const {
  chassis->set_heading_angle(headingangle(bytes, length));
  chassis->set_yaw_rate(yawrate(bytes, length));
  chassis->set_gps_velocity(gpsvelocity(bytes, length));
  chassis->set_gps_state(gpsstate(bytes, length));
}

// config detail: {'name': 'headingangle', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
// 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': 'degree'}
double Id0x713::headingangle(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'yawrate', 'offset': 0.0, 'precision': 0.01, 'len':
// 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]', 'bit': 32,
// 'type': 'double', 'order': 'intel', 'physical_unit': 'degree'}
double Id0x713::yawrate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 5);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 4);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'gpsvelocity', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': False, 'physical_range': '[0|655.35]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Id0x713::gpsvelocity(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'gpsstate', 'offset': 0.0, 'precision': 1.0, 'len':
// 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0, 'type':
// 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x713::gpsstate(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}
}  // namespace protocol
}  // namespace gps
}  // namespace drivers
}  // namespace apollo
