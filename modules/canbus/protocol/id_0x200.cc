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

#include "modules/canbus/protocol/id_0x200.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x200::ID = 0x200;

// public
Id0x200::Id0x200() { Reset(); }

uint32_t Id0x200::GetPeriod() const {
  static const uint32_t PERIOD = 200 * 1000;
  return PERIOD;
}

void Id0x200::UpdateData(uint8_t* data) {
  set_p_control_flag(data, control_flag_);
  set_p_stop_flag(data, stop_flag_);
  set_p_park_flag(data, park_flag_);
  set_p_wait_flag(data, wait_flag_);
  set_p_start_flag(data, start_flag_);
  set_p_mission_flag(data, mission_flag_);
  set_p_traj_number(data, traj_number_);
  set_p_gps_state(data, gps_state_);
  set_p_gear_position(data, gear_position_);
  set_p_speed_now(data, speed_now_);
  set_p_e_y_now(data, e_y_now_);
  set_p_e_phi_now(data, e_phi_now_);
}

void Id0x200::Reset() {
  control_flag_ = 0;
  stop_flag_ = 0;
  park_flag_ = 0;
  wait_flag_ = 0;
  start_flag_ = 0;
  mission_flag_ = 0;
  traj_number_ = 0;
  gps_state_ = 0;
  gear_position_ = 0;
  speed_now_ = 0;
  e_y_now_ = 0;
  e_phi_now_ = 0; 
}

Id0x200* Id0x200::set_control_flag(int control_flag) {
  control_flag_ = control_flag;
  return this;
}

void Id0x200::set_p_control_flag(uint8_t* data, int control_flag) {
  control_flag = ProtocolData::BoundedValue(0, 6, control_flag);
  int x = control_flag;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 2);
}

Id0x200* Id0x200::set_stop_flag(int stop_flag) {
  stop_flag_ = stop_flag;
  return this;
}

void Id0x200::set_p_stop_flag(uint8_t* data, int stop_flag) {
  stop_flag = ProtocolData::BoundedValue(0, 3, stop_flag);
  int x = stop_flag;

  Byte to_set(data + 0);
  to_set.set_value(x, 2, 2);
}

Id0x200* Id0x200::set_park_flag(int park_flag) {
  park_flag_ = park_flag;
  return this;
}

void Id0x200::set_p_park_flag(uint8_t* data, int park_flag) {
  park_flag = ProtocolData::BoundedValue(0, 1, park_flag);
  int x = park_flag;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 1);
}

Id0x200* Id0x200::set_wait_flag(int wait_flag) {
  wait_flag_ = wait_flag;
  return this;
}

void Id0x200::set_p_wait_flag(uint8_t* data, int wait_flag) {
  wait_flag = ProtocolData::BoundedValue(0, 1, wait_flag);
  int x = wait_flag;

  Byte to_set(data + 0);
  to_set.set_value(x, 5, 1);
}

Id0x200* Id0x200::set_start_flag(int start_flag) {
  start_flag_ = start_flag;
  return this;
}

void Id0x200::set_p_start_flag(uint8_t* data, int start_flag) {
  start_flag = ProtocolData::BoundedValue(0, 1, start_flag);
  int x = start_flag;

  Byte to_set(data + 0);
  to_set.set_value(x, 6, 1);
}

Id0x200* Id0x200::set_mission_flag(int mission_flag) {
  mission_flag_ = mission_flag;
  return this;
}

void Id0x200::set_p_mission_flag(uint8_t* data, int mission_flag) {
  mission_flag = ProtocolData::BoundedValue(0, 1, mission_flag);
  int x = mission_flag;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 1);
}

Id0x200* Id0x200::set_traj_number(int traj_number) {
  traj_number_ = traj_number;
  return this;
}

void Id0x200::set_p_traj_number(uint8_t* data, int traj_number) {
  traj_number = ProtocolData::BoundedValue(1, 2, traj_number);
  int x = traj_number;

  Byte to_set(data + 1);
  to_set.set_value(x, 1, 1);
}

Id0x200* Id0x200::set_gps_state(int gps_state) {
  gps_state_ = gps_state;
  return this;
}

void Id0x200::set_p_gps_state(uint8_t* data, int gps_state) {
  gps_state = ProtocolData::BoundedValue(0, 4, gps_state);
  int x = gps_state;

  Byte to_set(data + 1);
  to_set.set_value(x, 2, 3);
}

Id0x200* Id0x200::set_gear_position(int gear_position) {
  gear_position_ = gear_position;
  return this;
}

void Id0x200::set_p_gear_position(uint8_t* data, int gear_position) {
  gear_position = ProtocolData::BoundedValue(0, 4, gear_position);
  int x = gear_position;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 3);
}

Id0x200* Id0x200::set_speed_now(double speed_now) {
  speed_now_ = speed_now;
  return this;
}

void Id0x200::set_p_speed_now(uint8_t* data, double speed_now) {
  int x = speed_now / 0.01;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);

  x >>= 8;
  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0 ,8);
}

Id0x200* Id0x200::set_e_y_now(double e_y_now) {
  e_y_now_ = e_y_now;
  return this;
}

void Id0x200::set_p_e_y_now(uint8_t* data, double e_y_now) {
  int x = e_y_now / 0.01;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);

  x >>= 8;
  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0 ,8);
}

Id0x200* Id0x200::set_e_phi_now(double e_phi_now) {
  e_phi_now_ = e_phi_now;
  return this;
}

void Id0x200::set_p_e_phi_now(uint8_t* data, double e_phi_now) {
  int x = e_phi_now / 0.1;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 6);
  to_set0.set_value(t, 0, 8);

  x >>= 8;
  t = x & 0xFF;
  Byte to_set1(data + 7);
  to_set1.set_value(t, 0 ,8);
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
