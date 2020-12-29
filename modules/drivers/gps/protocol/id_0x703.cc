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

#include "modules/drivers/gps/protocol/id_0x703.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace TransportGPS {

using ::apollo::drivers::canbus::Byte;

Id0x703::Id0x703() {}
const int32_t Id0x703::ID = 0x703;

void Id0x703::Parse(const std::uint8_t* bytes, int32_t length,
                    apollo::drivers::Gps* chassis) const {
  chassis->set_velocity_down(velocitydown(bytes, length));
  chassis->set_velocity_lateral(velocitylateral(bytes, length));
  chassis->set_velocity_forward(velocityforward(bytes, length));
}

// config detail: {'name': 'velocitydown', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
// 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
double Id0x703::velocitydown(const std::uint8_t* bytes, int32_t length) const {
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

// config detail: {'name': 'velocitylateral', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
// 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
double Id0x703::velocitylateral(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'name': 'velocityforward', 'offset': 0.0, 'precision': 0.01,
// 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
// 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'm/s'}
double Id0x703::velocityforward(const std::uint8_t* bytes,
                                int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace TransportGPS
}  // namespace canbus
}  // namespace apollo
