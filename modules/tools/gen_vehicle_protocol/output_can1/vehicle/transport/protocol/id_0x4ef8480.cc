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

#include "modules/canbus/vehicle/Transport/protocol/id_0x4ef8480.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace Transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0x4ef8480::ID = 0x4EF8480;

// public
Id0x4ef8480::Id0x4ef8480() { Reset(); }

uint32_t Id0x4ef8480::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0x4ef8480::UpdateData(uint8_t* data) {
  set_p_vehiclesteerpumpreadysts(data, vehiclesteerpumpreadysts_);
  set_p_steerenablecmd(data, steerenablecmd_);
  set_p_steeranglespeedcmd(data, steeranglespeedcmd_);
  set_p_steeranglecmd(data, steeranglecmd_);
  set_p_lifecnt(data, lifecnt_);
  set_p_currentvehiclespeed(data, currentvehiclespeed_);
}

void Id0x4ef8480::Reset() {
  // TODO(All) :  you should check this manually
  vehiclesteerpumpreadysts_ = false;
  steerenablecmd_ = false;
  steeranglespeedcmd_ = 0.0;
  steeranglecmd_ = 0.0;
  lifecnt_ = 0;
  currentvehiclespeed_ = 0.0;
}

Id0x4ef8480* Id0x4ef8480::set_vehiclesteerpumpreadysts(
    bool vehiclesteerpumpreadysts) {
  vehiclesteerpumpreadysts_ = vehiclesteerpumpreadysts;
  return this;
 }

// config detail: {'name': 'VehicleSteerPumpReadySts', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_vehiclesteerpumpreadysts(uint8_t* data,
    bool vehiclesteerpumpreadysts) {
  int x = vehiclesteerpumpreadysts;

  Byte to_set(data + 0);
  to_set.set_value(x, 1, 1);
}


Id0x4ef8480* Id0x4ef8480::set_steerenablecmd(
    bool steerenablecmd) {
  steerenablecmd_ = steerenablecmd;
  return this;
 }

// config detail: {'name': 'SteerEnableCmd', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_steerenablecmd(uint8_t* data,
    bool steerenablecmd) {
  int x = steerenablecmd;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 1);
}


Id0x4ef8480* Id0x4ef8480::set_steeranglespeedcmd(
    double steeranglespeedcmd) {
  steeranglespeedcmd_ = steeranglespeedcmd;
  return this;
 }

// config detail: {'name': 'SteerAngleSpeedCmd', 'offset': -500.0, 'precision': 4.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|500]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_steeranglespeedcmd(uint8_t* data,
    double steeranglespeedcmd) {
  steeranglespeedcmd = ProtocolData::BoundedValue(0.0, 500.0, steeranglespeedcmd);
  int x = (steeranglespeedcmd - -500.000000) / 4.000000;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Id0x4ef8480* Id0x4ef8480::set_steeranglecmd(
    double steeranglecmd) {
  steeranglecmd_ = steeranglecmd;
  return this;
 }

// config detail: {'name': 'SteerAngleCmd', 'offset': -3276.7, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[-880|880]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_steeranglecmd(uint8_t* data,
    double steeranglecmd) {
  steeranglecmd = ProtocolData::BoundedValue(-880.0, 880.0, steeranglecmd);
  int x = (steeranglecmd - -3276.700000) / 0.100000;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 2);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 3);
  to_set1.set_value(t, 0, 8);
}


Id0x4ef8480* Id0x4ef8480::set_lifecnt(
    int lifecnt) {
  lifecnt_ = lifecnt;
  return this;
 }

// config detail: {'name': 'LifeCnt', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_lifecnt(uint8_t* data,
    int lifecnt) {
  lifecnt = ProtocolData::BoundedValue(0, 255, lifecnt);
  int x = lifecnt;

  Byte to_set(data + 7);
  to_set.set_value(x, 0, 8);
}


Id0x4ef8480* Id0x4ef8480::set_currentvehiclespeed(
    double currentvehiclespeed) {
  currentvehiclespeed_ = currentvehiclespeed;
  return this;
 }

// config detail: {'name': 'CurrentVehicleSpeed', 'offset': 0.0, 'precision': 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range': '[0|250.99]', 'bit': 32, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Id0x4ef8480::set_p_currentvehiclespeed(uint8_t* data,
    double currentvehiclespeed) {
  currentvehiclespeed = ProtocolData::BoundedValue(0.0, 250.99, currentvehiclespeed);
  int x = currentvehiclespeed / 0.003906;
  uint8_t t = 0;

  t = x & 0xFF;
  Byte to_set0(data + 4);
  to_set0.set_value(t, 0, 8);
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(data + 5);
  to_set1.set_value(t, 0, 8);
}

}  // namespace Transport
}  // namespace canbus
}  // namespace apollo
