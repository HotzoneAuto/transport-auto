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

class Id0x200 : public ::apollo::drivers::canbus::ProtocolData<
                        ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  Id0x200();

  uint32_t GetPeriod() const override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  Id0x200* set_control_flag(int control_flag);
  Id0x200* set_stop_flag(int stop_flag);
  Id0x200* set_park_flag(int park_flag);
  Id0x200* set_wait_flag(int wait_flag);
  Id0x200* set_start_flag(int start_flag);
  Id0x200* set_mission_flag(int mission_flag);
  Id0x200* set_traj_number(int traj_number);
  Id0x200* set_gps_state(int gps_state);
  Id0x200* set_gear_position(int gear_position);
  Id0x200* set_speed_now(double speed_now);
  Id0x200* set_e_y_now(double e_y_now);
  Id0x200* set_e_phi_now(double e_phi_now);


 private:
  void set_p_control_flag(uint8_t* data, int control_flag);
  void set_p_stop_flag(uint8_t* data, int stop_flag);
  void set_p_park_flag(uint8_t* data, int park_flag);
  void set_p_wait_flag(uint8_t* data, int wait_flag);
  void set_p_start_flag(uint8_t* data, int start_flag);
  void set_p_mission_flag(uint8_t* data, int mission_flag);
  void set_p_traj_number(uint8_t* data, int traj_number);
  void set_p_gps_state(uint8_t* data, int gps_state);
  void set_p_gear_position(uint8_t* data, int gear_position);
  void set_p_speed_now(uint8_t* data, double speed_now);
  void set_p_e_y_now(uint8_t* data, double e_y_now);
  void set_p_e_phi_now(uint8_t* data, double e_phi_now);



 private:
  int control_flag_;
  int stop_flag_;
  int park_flag_;
  int wait_flag_;
  int start_flag_;
  int mission_flag_;
  int traj_number_;
  int gps_state_;
  int gear_position_;
  double speed_now_;
  double e_y_now_;
  double e_phi_now_; 
};

}  // namespace transport
}  // namespace canbus
}  // namespace apollo
