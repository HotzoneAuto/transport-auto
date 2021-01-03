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

ErrorCode TransportController::Init(
    CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "TransportController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  /*
  // To be fixed, error says GetProtoConfig not declared.
  if (!GetProtoConfig(&transport_can_conf_)) {
    AERROR << "Unable to load config file!" << ConfigFilePath();
  }*/

  bool openconfig = apollo::cyber::common::GetProtoFromFile(
      "/apollo/modules/canbus/conf", &transport_can_conf_);
  if (!openconfig) {
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

  can_sender_->AddMessage(Id0x4ef8480::ID, id_0x4ef8480_, false);
  can_sender_->AddMessage(Id0xc040b2b::ID, id_0xc040b2b_, false);

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
    id_0x4ef8480_->set_steerenablecmd(1);
    id_0x4ef8480_->set_steeranglespeedcmd(350);
    id_0x4ef8480_->set_steeranglecmd(cmd.control_steer());
    id_0x4ef8480_->set_lifecnt(lifecnt);
    id_0x4ef8480_->set_currentvehiclespeed(5);
  }
  if (AccEnable == 1) {  //纵向控制启用
    AINFO << "Into Long control, vol_cur = " << vol_cur << ", vol_exp = " << vol_exp;

    // set system control mode as pedalopenreq mode, set 3 pedals require flag as 1
    id_0xc040b2b_->set_xbr1_sysctrlmode(2);
    id_0xc040b2b_->set_xbr1_accpedalreqflag(1);
    id_0xc040b2b_->set_xbr1_brkpedalreqflag(1);
    id_0xc040b2b_->set_xbr1_clupedalreqflag(1);

    int control_flag = 0;
    float ths_dif = transport_can_conf_.speederrorthreshold();
    float ths_exp = transport_can_conf_.speedthreshold();
    AINFO << "ths_exp = " << ths_exp << ", ths_dif = " << ths_dif
          << ", transport_can_conf_.idlespeed() = " << transport_can_conf_.idlespeed();
    AINFO << "transport_can_conf_.clutchset() = " << transport_can_conf_.clutchset();

    if ((vol_cur < transport_can_conf_.idlespeed()/2) && (vol_exp > ths_exp - 0.1)) {
      control_flag = 1;
      AINFO << "control_flag is set as: 1";
    } else if ((vol_cur < ths_exp) || (vol_exp - vol_cur) > ths_dif) {
      control_flag = 2;
      AINFO << "control_flag is set as: 2";
    } else if ((vol_exp - vol_cur) < ths_dif && (vol_cur > ths_exp)) {
      control_flag = 3;
      AINFO << "control_flag is set as: 3";
    } else if (vol_exp < transport_can_conf_.idlespeed()) {
      control_flag = 4;
      AINFO << "control_flag is set as: 4";
    }

    AINFO << "Before switch cases, control_flag = " << control_flag;
    if (control_flag) {
      switch (control_flag) {
        // start mode
        case 1:
          // brake set
          AINFO << "Into start mode, control_flag = " << control_flag;
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(0);
          id_0xc040b2b_->set_xbr1_accpedalopenreq(0);
          AINFO << "brkpedalopenreq and accpedalopenreq are set as: 0";
          // for(int i = clutchSet; i >= 0; i++) {
          for (int i = transport_can_conf_.clutchset() + int(5 * transport_can_conf_.clutchreleaserate()); i >= 0; i--) {
            // li he
            if (i > transport_can_conf_.clutchset()) {
                id_0xc040b2b_->set_xbr1_clupedalopenreq(transport_can_conf_.clutchset());
                AINFO << "clupedalopenreq is set as: " << transport_can_conf_.clutchset();
                std::this_thread::sleep_for(std::chrono::milliseconds(
                  int(1000 / transport_can_conf_.clutchreleaserate())));
            } else {
                id_0xc040b2b_->set_xbr1_clupedalopenreq(i);
                AINFO << "clupedalopenreq is set as: " << i;
                std::this_thread::sleep_for(std::chrono::milliseconds(
                  int(1000 / transport_can_conf_.clutchreleaserate())));
            }
          }
          break;
        // normal mode
        case 2:
          // P control
          AINFO << "Into normal mode, control_flag = " << control_flag;
          id_0xc040b2b_->set_xbr1_accpedalopenreq(
              int(vol_exp / transport_can_conf_.kspeedthrottle() + (vol_exp - vol_cur) * transport_can_conf_.kdrive()));
          AINFO << "accpedalopenreq is set as: " << int(vol_exp / transport_can_conf_.kspeedthrottle() + (vol_exp - vol_cur) * transport_can_conf_.kdrive());
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(0);
          id_0xc040b2b_->set_xbr1_clupedalopenreq(0);
          AINFO << "brkpedalopenreq and clupedalopenreq are set as: 0";
          break;
        // emergency mode
        case 3:
          // P control
          AINFO << "Into emergency mode, control_flag = " << control_flag;
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(
              int(- (vol_exp - vol_cur) * transport_can_conf_.kbrake()));
          AINFO << "brkpedalopenreq is set as: " << int(- (vol_exp - vol_cur) * transport_can_conf_.kbrake());
          id_0xc040b2b_->set_xbr1_accpedalopenreq(0);
          id_0xc040b2b_->set_xbr1_clupedalopenreq(0);
          AINFO << "accpedalopenreq and clupedalopenreq are set as: 0";
          break;
        // stop mode
        case 4:
          // li he
          // id_0xc040b2b_->set_xbr1_brkpedalopenreq(clutchSet);
          AINFO << "Into stop mode, control_flag = " << control_flag;
          id_0xc040b2b_->set_xbr1_clupedalopenreq(transport_can_conf_.clutchset());
          AINFO << "clupedalopenreq is set as: " << transport_can_conf_.clutchset();
          // for(int i = brakeSet; i <= 100; i++) {
          /* for (int i = 0; i <= transport_can_conf_.brakeset(); i++) {
            id_0xc040b2b_->set_xbr1_brkpedalopenreq(i);
            AINFO << "brkpedalopenreq is set as: " << i;
            std::this_thread::sleep_for(std::chrono::milliseconds(
                int(1000 / transport_can_conf_.brakeapplyrate())));
          }*/
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(transport_can_conf_.brakeset());
          AINFO << "brkpedalopenreq is set as: " << transport_can_conf_.brakeset();
          id_0xc040b2b_->set_xbr1_accpedalopenreq(0);
          AINFO << "accpedalopenreq is set as: " << 0;
          break;
        default:
          AINFO << "In default, control_flag = " << control_flag;
          break;
      }

      if (count_flag == 15) {
        control_flag = 0;
      } else {
        count_flag++;
      }
      id_0xc040b2b_->set_xbr1_rollingcnt(count_flag);
    }
  }
}

TransportController::~TransportController() {}
