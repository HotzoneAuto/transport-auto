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

class Id0x284 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x284();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Id0x284* set_transport_state(int transport_state);

  Id0x284* set_receive_digger_gps_flag(int receive_digger_gps_flag);

 private:
  void set_p_transport_state(uint8_t* data, int transport_state);

  void set_p_receive_digger_gps_flag(uint8_t* data, int receive_digger_gps_flag);

 private:
  int transport_state_;
  int receive_digger_gps_flag_;
};

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
