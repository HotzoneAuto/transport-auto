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

#include "modules/canbus/protocol/id_0x302.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x302::ID = 0x302;

// public
Id0x302::Id0x302() { Reset(); }

uint32_t Id0x302::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Id0x302::UpdateData(uint8_t* data) {
  set_p_infotodigger_flag(data, infotodigger_flag_);
  set_p_ackloctodigger_flag(data, ackloctodigger_flag_);
  set_p_vehheighttodigger(data, vehheighttodigger_);
}

void Id0x302::Reset() {
  infotodigger_flag_ = 0;
  ackloctodigger_flag_ = 0;
  vehheighttodigger_ = 0;
}

Id0x302* Id0x302::set_infotodigger_flag(int infotodigger_flag) {
  infotodigger_flag_ = infotodigger_flag;
  return this;
}

void Id0x302::set_p_infotodigger_flag(uint8_t* data, int infotodigger_flag) {
  infotodigger_flag = ProtocolData::BoundedValue(0, 15, infotodigger_flag);
  int x = infotodigger_flag;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 4);
}

Id0x302* Id0x302::set_ackloctodigger_flag(int ackloctodigger_flag) {
  ackloctodigger_flag_ = ackloctodigger_flag;
  return this;
}

void Id0x302::set_p_ackloctodigger_flag(uint8_t* data, int ackloctodigger_flag) {
  ackloctodigger_flag = ProtocolData::BoundedValue(0, 1, ackloctodigger_flag);
  int x = ackloctodigger_flag;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Id0x302* Id0x302::set_vehheighttodigger(double vehheighttodigger) {
  vehheighttodigger_ = vehheighttodigger;
  return this;
}

void Id0x302::set_p_vehheighttodigger(uint8_t* data, double vehheighttodigger) {
  vehheighttodigger = ProtocolData::BoundedValue(0.00, 5.10, vehheighttodigger);
  int x = vehheighttodigger / 0.02;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
