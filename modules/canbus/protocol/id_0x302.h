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

class Id0x302 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x302();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Id0x302* set_infotodigger_flag(int infotodigger_flag);

  Id0x302* set_ackloctodigger_flag(int ackloctodigger_flag);

  Id0x302* set_vehheighttodigger(double vehheighttodigger);

 private:
  void set_p_infotodigger_flag(uint8_t* data, int infotodigger_flag);

  void set_p_ackloctodigger_flag(uint8_t* data, int ackloctodigger_flag);

  void set_p_vehheighttodigger(uint8_t* data, double vehheighttodigger);

 private:
  int infotodigger_flag_;
  int ackloctodigger_flag_;
  double vehheighttodigger_;
};

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
