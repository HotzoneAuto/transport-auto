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

#pragma once

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/gps/proto/gps.pb.h"

namespace apollo {
namespace drivers {
namespace gps {
namespace protocol {

class Id0x712
    : public ::apollo::drivers::canbus::ProtocolData< ::apollo::drivers::Gps> {
 public:
  static const int32_t ID;
  Id0x712();
  void Parse(const std::uint8_t* bytes, int32_t length,
             apollo::drivers::Gps* chassis) const override;

 private:
  // config detail: {'name': 'GPSNH', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int gpsnh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GPSNL', 'offset': 0.0, 'precision': 1e-07, 'len':
  // 24, 'is_signed_var': False, 'physical_range': '[0|1.6777215]', 'bit': 32,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double gpsnl(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GPSEH', 'offset': 0.0, 'precision': 1.0, 'len': 8,
  // 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 24, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  int gpseh(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GPSEL', 'offset': 0.0, 'precision': 1e-07, 'len':
  // 24, 'is_signed_var': False, 'physical_range': '[0|1.6777215]', 'bit': 0,
  // 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double gpsel(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace protocol
}  // namespace gps
}  // namespace drivers
}  // namespace apollo
