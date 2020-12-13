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

#include "modules/transport_can/vehicle/transportgps/protocol/id_0x712.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace TransportGPS {

using ::apollo::drivers::canbus::Byte;

Id0x712::Id0x712() {}
const int32_t Id0x712::ID = 0x712;

void Id0x712::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->set_gpsnh(gpsnh(bytes, length));
  chassis->set_gpsnl(gpsnl(bytes, length));
  chassis->set_gpseh(gpseh(bytes, length));
  chassis->set_gpsel(gpsel(bytes, length));
}

// config detail: {'name': 'gpsnh', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x712::gpsnh(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 7);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'gpsnl', 'offset': 0.0, 'precision': 1e-07, 'len': 24, 'is_signed_var': False, 'physical_range': '[0|1.6777215]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Id0x712::gpsnl(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 6);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 5);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 4);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000000;
  return ret;
}

// config detail: {'name': 'gpseh', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
int Id0x712::gpseh(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'name': 'gpsel', 'offset': 0.0, 'precision': 1e-07, 'len': 24, 'is_signed_var': False, 'physical_range': '[0|1.6777215]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
double Id0x712::gpsel(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 1);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  Byte t2(bytes + 0);
  t = t2.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.000000;
  return ret;
}
}  // namespace TransportGPS
}  // namespace canbus
}  // namespace apollo
