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
#include "modules/transport_can/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace Transport {

class Id0x4ef8480 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x4ef8480();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'VehicleSteerPumpReadySts', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Id0x4ef8480* set_vehiclesteerpumpreadysts(bool vehiclesteerpumpreadysts);

  // config detail: {'name': 'SteerEnableCmd', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Id0x4ef8480* set_steerenablecmd(bool steerenablecmd);

  // config detail: {'name': 'SteerAngleSpeedCmd', 'offset': -500.0,
  // 'precision': 4.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|500]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  Id0x4ef8480* set_steeranglespeedcmd(double steeranglespeedcmd);

  // config detail: {'name': 'SteerAngleCmd', 'offset': -3276.7, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[-880|880]',
  // 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Id0x4ef8480* set_steeranglecmd(double steeranglecmd);

  // config detail: {'name': 'LifeCnt', 'offset': 0.0, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  Id0x4ef8480* set_lifecnt(int lifecnt);

  // config detail: {'name': 'CurrentVehicleSpeed', 'offset': 0.0, 'precision':
  // 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|250.99]', 'bit': 32, 'type': 'double', 'order': 'intel',
  // 'physical_unit': ''}
  Id0x4ef8480* set_currentvehiclespeed(double currentvehiclespeed);

 private:
  // config detail: {'name': 'VehicleSteerPumpReadySts', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_vehiclesteerpumpreadysts(uint8_t* data,
                                      bool vehiclesteerpumpreadysts);

  // config detail: {'name': 'SteerEnableCmd', 'offset': 0.0, 'precision': 1.0,
  // 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
  // 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_steerenablecmd(uint8_t* data, bool steerenablecmd);

  // config detail: {'name': 'SteerAngleSpeedCmd', 'offset': -500.0,
  // 'precision': 4.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|500]', 'bit': 8, 'type': 'double', 'order': 'intel', 'physical_unit':
  // ''}
  void set_p_steeranglespeedcmd(uint8_t* data, double steeranglespeedcmd);

  // config detail: {'name': 'SteerAngleCmd', 'offset': -3276.7, 'precision':
  // 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[-880|880]',
  // 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_steeranglecmd(uint8_t* data, double steeranglecmd);

  // config detail: {'name': 'LifeCnt', 'offset': 0.0, 'precision': 1.0, 'len':
  // 8, 'is_signed_var': False, 'physical_range': '[0|255]', 'bit': 56, 'type':
  // 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_lifecnt(uint8_t* data, int lifecnt);

  // config detail: {'name': 'CurrentVehicleSpeed', 'offset': 0.0, 'precision':
  // 0.00390625, 'len': 16, 'is_signed_var': False, 'physical_range':
  // '[0|250.99]', 'bit': 32, 'type': 'double', 'order': 'intel',
  // 'physical_unit': ''}
  void set_p_currentvehiclespeed(uint8_t* data, double currentvehiclespeed);

 private:
  bool vehiclesteerpumpreadysts_;
  bool steerenablecmd_;
  double steeranglespeedcmd_;
  double steeranglecmd_;
  int lifecnt_;
  double currentvehiclespeed_;
};

}  // namespace Transport
}  // namespace canbus
}  // namespace apollo
