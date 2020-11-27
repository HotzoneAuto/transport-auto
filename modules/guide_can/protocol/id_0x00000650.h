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
#include "modules/guide_can/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace guide {

class Id0x00000650 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x00000650();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'UWB_zitai', 'offset': 0.0, 'precision': 1.0,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 48,
  // 'type': 'int', 'order': 'intel', 'physical_unit': 'deg'}
  int uwb_attitude(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UWB_fangwei', 'offset': 0.0, 'precision': 1.0,
  // 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 32,
  // 'type': 'int', 'order': 'intel', 'physical_unit': 'deg'}
  int uwb_azimuth(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UWB_distance', 'offset': 0.0, 'precision': 0.01,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16,
  // 'type': 'double', 'order': 'intel', 'physical_unit': 'm'}
  double uwb_distance(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'UWB_status', 'offset': 0.0, 'precision': 1.0,
  // 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8,
  // 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int uwb_status(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
