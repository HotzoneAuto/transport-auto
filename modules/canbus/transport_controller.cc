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

#include "modules/canbus/transport_controller.h"

DEFINE_string(can_conf_file,
              "/apollo/modules/canbus/conf/transport_can_conf.pb",
              "default control conf data file.");

namespace apollo {
namespace canbus {

ErrorCode TransportController::Init(
    CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "TransportController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (!apollo::cyber::common::GetProtoFromFile(
      FLAGS_can_conf_file, &transport_can_conf_)) {
    AERROR << "Unbale to load transport can conf file!";
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  id_0x4ef8480_ = dynamic_cast<Id0x4ef8480 *>(
      message_manager_->GetMutableProtocolDataById(Id0x4ef8480::ID));
  if (id_0x4ef8480_ == nullptr) {
    AERROR << "Id0x4ef8480 does not exist in the TransportMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  id_0xc040b2b_ = dynamic_cast<Id0xc040b2b *>(
      message_manager_->GetMutableProtocolDataById(Id0xc040b2b::ID));
  if (id_0xc040b2b_ == nullptr) {
    AERROR << "Id0xc040b2b does not exist in the TransportMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  id_0x302_ = dynamic_cast<Id0x302 *>(
      message_manager_->GetMutableProtocolDataById(Id0x302::ID));
  if (id_0x302_ == nullptr) {
    AERROR << "Id0x302 does not exist in the TransportMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  
  id_0x1314_ = dynamic_cast<Id0x1314 *>(
      message_manager_->GetMutableProtocolDataById(Id0x1314::ID));
  if (id_0x1314_ == nullptr) {
    AERROR << "Id0x1314 does not exist in the TransportMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  id_0x200_ = dynamic_cast<Id0x200 *>(
      message_manager_->GetMutableProtocolDataById(Id0x200::ID));
  if (id_0x200_ == nullptr) {
    AERROR << "Id0x200 does not exist in the TransportMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Id0x4ef8480::ID, id_0x4ef8480_, false);
  can_sender_->AddMessage(Id0xc040b2b::ID, id_0xc040b2b_, false);
  can_sender_->AddMessage(Id0x302::ID, id_0x302_, false);
  can_sender_->AddMessage(Id0x1314::ID, id_0x1314_, false);
  can_sender_->AddMessage(Id0x200::ID, id_0x200_, false);

  // need sleep to ensure all messages received
  AINFO << "TransportController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

void TransportController::Start() {
  if (!is_initialized_) {
    AERROR << "Controller didn't Init";
    return;
  }
  if (!is_start)
    is_start = true;
  else
    AERROR << "Duplicate Start ";
}
void TransportController::Stop() {
  if (!is_initialized_) {
    AERROR << "Controller didn't Init";
    return;
  }
  if (is_start) is_start = false;
}

// After digger sends signal, change gps traj in control, reinitialize paras
// here.
void TransportController::ControlUpdate(ControlCommand cmd,
                                        const int SteerEnable,
                                        const int AccEnable, float vol_cur,
                                        float vol_exp) {
  if (!is_start) {
    AERROR << "Controller didn't start";
    return;
  }

  if (SteerEnable == 1) {  //横向控制启用
    static int lifecnt = 0;
    lifecnt++;
    if (lifecnt > 255) lifecnt = 0;
    id_0x4ef8480_->set_steerenablecmd(cmd.control_steer_flag());
    //TODO: config variable
    id_0x4ef8480_->set_steeranglespeedcmd(cmd.control_steeranglespeed());
    id_0x4ef8480_->set_steeranglecmd(cmd.control_steer());
    id_0x4ef8480_->set_lifecnt(lifecnt);
    id_0x4ef8480_->set_currentvehiclespeed(5);
  }

  if (AccEnable == 1) {  //纵向控制启用
    AINFO << "Into Long control, vol_cur = " << vol_cur
          << ", vol_exp = " << vol_exp;

    // set system control mode as pedalopenreq mode, set 3 pedals require flag
    // as 1
    id_0xc040b2b_->set_xbr1_sysctrlmode(2);
    id_0xc040b2b_->set_xbr1_vehaccreq(-15);

    id_0xc040b2b_->set_xbr1_accpedalreqflag(cmd.control_accpedal_flag());
    id_0xc040b2b_->set_xbr1_brkpedalreqflag(cmd.control_brkpedal_flag());
    id_0xc040b2b_->set_xbr1_clupedalreqflag(cmd.control_clupedal_flag());

    id_0xc040b2b_->set_xbr1_accpedalopenreq(cmd.control_accpedal());
    id_0xc040b2b_->set_xbr1_brkpedalopenreq(cmd.control_brkpedal());
    id_0xc040b2b_->set_xbr1_clupedalopenreq(cmd.control_clupedal());

    id_0x1314_->set_gearshiftcmd(cmd.control_gear());

    if (count_flag == 15) {
      count_flag = 0;
    } else {
      count_flag++;
    }    
    id_0xc040b2b_->set_xbr1_rollingcnt(count_flag);  
  }
}

void TransportController::FlagUpdate(ControlFlag controlflag) {
  if (!is_start) {
    AERROR << "Controller didn't start";
    return;
  }
  id_0x302_->set_infotodigger_flag(controlflag.statetodigger_flag());
  id_0x302_->set_ackloctodigger_flag(controlflag.gpsacktodigger_flag());
 
}

void TransportController::RemoteUpdate(ControlToRemote controltoremote) {
  if (!is_start) {
    AERROR << "Controller didn't start";
    return;
  }
  id_0x200_->set_control_flag(controltoremote.control_flag());
  id_0x200_->set_wait_flag(controltoremote.wait_flag());
  id_0x200_->set_start_flag(controltoremote.start_flag());
  id_0x200_->set_park_flag(controltoremote.park_flag());
  id_0x200_->set_stop_flag(controltoremote.stop_flag());
  id_0x200_->set_mission_flag(controltoremote.mission_flag());
  id_0x200_->set_traj_number(controltoremote.traj_number());

  id_0x200_->set_gps_state(controltoremote.gps_state());
  id_0x200_->set_speed_now(controltoremote.gps_velocity());

  id_0x200_->set_gear_position(controltoremote.gear_position());

  id_0x200_->set_e_y_now(controltoremote.e_y());
  id_0x200_->set_e_phi_now(controltoremote.e_phi());
}

TransportController::~TransportController() {}
}  // namespace canbus
}  // namespace apollo
