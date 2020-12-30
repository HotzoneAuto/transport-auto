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

#include "modules/transport_can/protocol/id_0xc040b2b.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace transport {

using ::apollo::drivers::canbus::Byte;

const int32_t Id0xc040b2b::ID = 0xC040B2B;

// public
Id0xc040b2b::Id0xc040b2b() { Reset(); }

uint32_t Id0xc040b2b::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Id0xc040b2b::UpdateData(uint8_t* data) {
  set_p_xbr1_vehaccreqmode(data, xbr1_vehaccreqmode_);
  set_p_xbr1_vehaccreq(data, xbr1_vehaccreq_);
  set_p_xbr1_clupedalreqflag(data, xbr1_clupedalreqflag_);
  set_p_xbr1_clupedalopenreq(data, xbr1_clupedalopenreq_);
  set_p_xbr1_brkpedalreqflag(data, xbr1_brkpedalreqflag_);
  set_p_xbr1_brkpedalopenreq(data, xbr1_brkpedalopenreq_);
  set_p_xbr1_accpedalreqflag(data, xbr1_accpedalreqflag_);
  set_p_xbr1_accpedalopenreq(data, xbr1_accpedalopenreq_);
  set_p_xbr1_rollingcnt(data, xbr1_rollingcnt_);
  set_p_xbr1_sysctrlmode(data, xbr1_sysctrlmode_);
}

void Id0xc040b2b::Reset() {
  // TODO(All) :  you should check this manually
  xbr1_vehaccreqmode_ = 0;
  xbr1_vehaccreq_ = 0.0;
  xbr1_clupedalreqflag_ = false;
  xbr1_clupedalopenreq_ = 0;
  xbr1_brkpedalreqflag_ = false;
  xbr1_brkpedalopenreq_ = 0;
  xbr1_accpedalreqflag_ = false;
  xbr1_accpedalopenreq_ = 0;
  xbr1_rollingcnt_ = 0;
  xbr1_sysctrlmode_ = 0;
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_vehaccreqmode(int xbr1_vehaccreqmode) {
  xbr1_vehaccreqmode_ = xbr1_vehaccreqmode;
  return this;
}

// config detail: {'name': 'XBR1_VehAccReqMode', 'offset': 0.0,
// 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range':
// '[0|2]', 'bit': 8, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_vehaccreqmode(uint8_t* data,
                                           int xbr1_vehaccreqmode) {
  xbr1_vehaccreqmode = ProtocolData::BoundedValue(0, 2, xbr1_vehaccreqmode);
  int x = xbr1_vehaccreqmode;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 4);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_vehaccreq(double xbr1_vehaccreq) {
  xbr1_vehaccreq_ = xbr1_vehaccreq;
  return this;
}

// config detail: {'name': 'XBR1_VehAccReq', 'offset': -15.0, 'precision': 0.1,
// 'len': 8, 'is_signed_var': True, 'physical_range': '[-15|15]', 'bit': 16,
// 'type': 'double', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_vehaccreq(uint8_t* data, double xbr1_vehaccreq) {
  xbr1_vehaccreq = ProtocolData::BoundedValue(-15.0, 15.0, xbr1_vehaccreq);
  int x = (xbr1_vehaccreq - -15.000000) / 0.100000;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_clupedalreqflag(bool xbr1_clupedalreqflag) {
  xbr1_clupedalreqflag_ = xbr1_clupedalreqflag;
  return this;
}

// config detail: {'name': 'XBR1_CluPedalReqFlag', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_clupedalreqflag(uint8_t* data,
                                             bool xbr1_clupedalreqflag) {
  int x = xbr1_clupedalreqflag;

  Byte to_set(data + 1);
  to_set.set_value(x, 6, 1);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_clupedalopenreq(int xbr1_clupedalopenreq) {
  xbr1_clupedalopenreq_ = xbr1_clupedalopenreq;
  return this;
}

// config detail: {'name': 'XBR1_CluPedalOpenReq', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 48, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_clupedalopenreq(uint8_t* data,
                                             int xbr1_clupedalopenreq) {
  xbr1_clupedalopenreq =
      ProtocolData::BoundedValue(0, 255, xbr1_clupedalopenreq);
  int x = xbr1_clupedalopenreq;

  Byte to_set(data + 6);
  to_set.set_value(x, 0, 8);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_brkpedalreqflag(bool xbr1_brkpedalreqflag) {
  xbr1_brkpedalreqflag_ = xbr1_brkpedalreqflag;
  return this;
}

// config detail: {'name': 'XBR1_BrkPedalReqFlag', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_brkpedalreqflag(uint8_t* data,
                                             bool xbr1_brkpedalreqflag) {
  int x = xbr1_brkpedalreqflag;

  Byte to_set(data + 1);
  to_set.set_value(x, 5, 1);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_brkpedalopenreq(int xbr1_brkpedalopenreq) {
  xbr1_brkpedalopenreq_ = xbr1_brkpedalopenreq;
  return this;
}

// config detail: {'name': 'XBR1_BrkPedalOpenReq', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 40, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_brkpedalopenreq(uint8_t* data,
                                             int xbr1_brkpedalopenreq) {
  xbr1_brkpedalopenreq =
      ProtocolData::BoundedValue(0, 255, xbr1_brkpedalopenreq);
  int x = xbr1_brkpedalopenreq;

  Byte to_set(data + 5);
  to_set.set_value(x, 0, 8);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_accpedalreqflag(bool xbr1_accpedalreqflag) {
  xbr1_accpedalreqflag_ = xbr1_accpedalreqflag;
  return this;
}

// config detail: {'name': 'XBR1_AccPedalReqFlag', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_accpedalreqflag(uint8_t* data,
                                             bool xbr1_accpedalreqflag) {
  int x = xbr1_accpedalreqflag;

  Byte to_set(data + 1);
  to_set.set_value(x, 4, 1);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_accpedalopenreq(int xbr1_accpedalopenreq) {
  xbr1_accpedalopenreq_ = xbr1_accpedalopenreq;
  return this;
}

// config detail: {'name': 'XBR1_AccPedalOpenReq', 'offset': 0.0,
// 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range':
// '[0|255]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_accpedalopenreq(uint8_t* data,
                                             int xbr1_accpedalopenreq) {
  xbr1_accpedalopenreq =
      ProtocolData::BoundedValue(0, 255, xbr1_accpedalopenreq);
  int x = xbr1_accpedalopenreq;

  Byte to_set(data + 4);
  to_set.set_value(x, 0, 8);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_rollingcnt(int xbr1_rollingcnt) {
  xbr1_rollingcnt_ = xbr1_rollingcnt;
  return this;
}

// config detail: {'name': 'XBR1_RollingCnt', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0|15]', 'bit': 4,
// 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_rollingcnt(uint8_t* data, int xbr1_rollingcnt) {
  xbr1_rollingcnt = ProtocolData::BoundedValue(0, 15, xbr1_rollingcnt);
  int x = xbr1_rollingcnt;

  Byte to_set(data + 0);
  to_set.set_value(x, 4, 4);
}

Id0xc040b2b* Id0xc040b2b::set_xbr1_sysctrlmode(int xbr1_sysctrlmode) {
  xbr1_sysctrlmode_ = xbr1_sysctrlmode;
  return this;
}

// config detail: {'name': 'XBR1_SysCtrlMode', 'offset': 0.0, 'precision': 1.0,
// 'len': 4, 'is_signed_var': False, 'physical_range': '[0|3]', 'bit': 0,
// 'type': 'int', 'order': 'intel', 'physical_unit': ''}
void Id0xc040b2b::set_p_xbr1_sysctrlmode(uint8_t* data, int xbr1_sysctrlmode) {
  xbr1_sysctrlmode = ProtocolData::BoundedValue(0, 3, xbr1_sysctrlmode);
  int x = xbr1_sysctrlmode;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 4);
}

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
