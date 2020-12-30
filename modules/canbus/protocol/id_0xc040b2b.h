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
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace transport {

class Id0xc040b2b : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0xc040b2b();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'name': 'XBR1_VehAccReqMode', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|2]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_vehaccreqmode(int xbr1_vehaccreqmode);

  // config detail: {'name': 'XBR1_VehAccReq', 'offset': -15.0, 'precision':
  // 0.1, 'len': 8, 'is_signed_var': True, 'physical_range': '[-15|15]', 'bit':
  // 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_vehaccreq(double xbr1_vehaccreq);

  // config detail: {'name': 'XBR1_CluPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_clupedalreqflag(bool xbr1_clupedalreqflag);

  // config detail: {'name': 'XBR1_CluPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_clupedalopenreq(int xbr1_clupedalopenreq);

  // config detail: {'name': 'XBR1_BrkPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_brkpedalreqflag(bool xbr1_brkpedalreqflag);

  // config detail: {'name': 'XBR1_BrkPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_brkpedalopenreq(int xbr1_brkpedalopenreq);

  // config detail: {'name': 'XBR1_AccPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_accpedalreqflag(bool xbr1_accpedalreqflag);

  // config detail: {'name': 'XBR1_AccPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_accpedalopenreq(int xbr1_accpedalopenreq);

  // config detail: {'name': 'XBR1_RollingCnt', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 4,
  // 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_rollingcnt(int xbr1_rollingcnt);

  // config detail: {'name': 'XBR1_SysCtrlMode', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  Id0xc040b2b* set_xbr1_sysctrlmode(int xbr1_sysctrlmode);

 private:
  // config detail: {'name': 'XBR1_VehAccReqMode', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|2]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_vehaccreqmode(uint8_t* data, int xbr1_vehaccreqmode);

  // config detail: {'name': 'XBR1_VehAccReq', 'offset': -15.0, 'precision':
  // 0.1, 'len': 8, 'is_signed_var': True, 'physical_range': '[-15|15]', 'bit':
  // 16, 'type': 'double', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_vehaccreq(uint8_t* data, double xbr1_vehaccreq);

  // config detail: {'name': 'XBR1_CluPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_clupedalreqflag(uint8_t* data, bool xbr1_clupedalreqflag);

  // config detail: {'name': 'XBR1_CluPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_clupedalopenreq(uint8_t* data, int xbr1_clupedalopenreq);

  // config detail: {'name': 'XBR1_BrkPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_brkpedalreqflag(uint8_t* data, bool xbr1_brkpedalreqflag);

  // config detail: {'name': 'XBR1_BrkPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_brkpedalopenreq(uint8_t* data, int xbr1_brkpedalopenreq);

  // config detail: {'name': 'XBR1_AccPedalReqFlag', 'offset': 0.0,
  // 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
  // '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_accpedalreqflag(uint8_t* data, bool xbr1_accpedalreqflag);

  // config detail: {'name': 'XBR1_AccPedalOpenReq', 'offset': 0.0,
  // 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
  // '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_accpedalopenreq(uint8_t* data, int xbr1_accpedalopenreq);

  // config detail: {'name': 'XBR1_RollingCnt', 'offset': 0.0, 'precision': 1.0,
  // 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 4,
  // 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_rollingcnt(uint8_t* data, int xbr1_rollingcnt);

  // config detail: {'name': 'XBR1_SysCtrlMode', 'offset': 0.0,
  // 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
  // '[0|3]', 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  void set_p_xbr1_sysctrlmode(uint8_t* data, int xbr1_sysctrlmode);

 private:
  int xbr1_vehaccreqmode_;
  double xbr1_vehaccreq_;
  bool xbr1_clupedalreqflag_;
  int xbr1_clupedalopenreq_;
  bool xbr1_brkpedalreqflag_;
  int xbr1_brkpedalopenreq_;
  bool xbr1_accpedalreqflag_;
  int xbr1_accpedalopenreq_;
  int xbr1_rollingcnt_;
  int xbr1_sysctrlmode_;
};

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
