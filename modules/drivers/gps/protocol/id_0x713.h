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

class Id0x713
    : public ::apollo::drivers::canbus::ProtocolData< ::apollo::drivers::Gps> {
 public:
  static const int32_t ID;
  Id0x713();
  void Parse(const std::uint8_t* bytes, int32_t length,
             apollo::drivers::Gps* chassis) const override;

 private:
  // config detail: {'name': 'HeadingAngle', 'offset': 0.0, 'precision': 0.01,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]',
  // 'bit': 48, 'type': 'double', 'order': 'intel', 'physical_unit': 'degree'}
  double headingangle(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'YawRate', 'offset': 0.0, 'precision': 0.01, 'len':
  // 16, 'is_signed_var': True, 'physical_range': '[-327.68|327.67]', 'bit': 32,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'degree'}
  double yawrate(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GPSvelocity', 'offset': 0.0, 'precision': 0.01,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|655.35]', 'bit':
  // 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  double gpsvelocity(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'GPSstate', 'offset': 0.0, 'precision': 1.0, 'len':
  // 16, 'is_signed_var': False, 'physical_range': '[0|65535]', 'bit': 0,
  // 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int gpsstate(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace protocol
}  // namespace gps
}  // namespace drivers
}  // namespace apollo
