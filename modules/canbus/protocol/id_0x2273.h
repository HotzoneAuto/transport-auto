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

#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transport {

class Id0x2273 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x2273();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Id0x2273* set_stopfrmaster_flag(int stopfrmaster_flag);

  Id0x2273* set_missionfrmaster_flag(int missionfrmaster_flag);

  Id0x2273* set_trajfrmaster_flag(int trajfrmaster_flag);

 private:
  void set_p_stopfrmaster_flag(uint8_t* data, int stopfrmaster_flag);

  void set_p_missionfrmaster_flag(uint8_t* data, int missionfrmaster_flag);

  void set_p_trajfrmaster_flag(uint8_t* data, int trajfrmaster_flag);

 private:
  int stopfrmaster_flag_;
  int missionfrmaster_flag_;
  int trajfrmaster_flag_;
};

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
