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

#include "modules/canbus/protocol/id_0x284.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x284::ID = 0x284;

// public
Id0x284::Id0x284() { Reset(); }

uint32_t Id0x284::GetPeriod() const {
  static const uint32_t PERIOD = 100 * 1000;
  return PERIOD;
}

void Id0x284::UpdateData(uint8_t* data) {
  set_p_transport_state(data, transport_state_);
  set_p_receive_digger_gps_flag(data, receive_digger_gps_flag_);
}

void Id0x284::Reset() {
  transport_state_ = 0;
  receive_digger_gps_flag_ = 0;
}

Id0x284* Id0x284::set_transport_state(int transport_state) {
  transport_state_ = transport_state;
  return this;
}

void Id0x284::set_p_transport_state(uint8_t* data, int transport_state) {
  transport_state = ProtocolData::BoundedValue(0, 15, transport_state);
  int x = transport_state;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 4);
}

Id0x284* Id0x284::set_receive_digger_gps_flag(int receive_digger_gps_flag) {
  receive_digger_gps_flag_ = receive_digger_gps_flag;
  return this;
}

void Id0x284::set_p_receive_digger_gps_flag(uint8_t* data, int receive_digger_gps_flag) {
  receive_digger_gps_flag = ProtocolData::BoundedValue(0, 1, receive_digger_gps_flag);
  int x = receive_digger_gps_flag;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Id0x284* Id0x284::set_transport_height(double transport_height) {
  transport_height_ = transport_height;
  return this;
}

void Id0x284::set_p_transport_height(uint8_t* data, double transport_height) {
  transport_height = ProtocolData::BoundedValue(0.00, 5.10, transport_height);
  int x = transport_height / 0.02;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
