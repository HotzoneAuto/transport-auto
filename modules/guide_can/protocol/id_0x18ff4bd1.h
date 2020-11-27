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

class Id0x18ff4bd1 : public ::apollo::drivers::canbus::ProtocolData<
                         ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Id0x18ff4bd1();
  void Parse(const std::uint8_t* bytes, int32_t length,
             ChassisDetail* chassis) const override;

 private:
  // config detail: {'name': 'Steer_angle', 'offset': -3276.7, 'precision': 0.1,
  // 'len': 16, 'is_signed_var': False, 'physical_range': '[-1260|1260]', 'bit':
  // 0, 'type': 'double', 'order': 'intel', 'physical_unit': 'deg'}
  double steer_angle(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace guide
}  // namespace canbus
}  // namespace apollo
