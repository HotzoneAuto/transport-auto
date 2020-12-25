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

#include "modules/transport_can/vehicle/transport/transport_controller.h"

ErrorCode TransportController::Init(
    CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "TransportController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  bool openconfig = apollo::cyber::common::GetProtoFromFile(
                                        "/apollo/modules/transport_can/conf", 
                                        &transport_can_conf_);
  if (!openconfig) {
    AERROR << "Unbale to load transport can conf file!";
  }

  // f.open("/apollo/modules/transport_can/conf/transport_can.conf");
  // if (f.is_open()) {
  //   AINFO << "Control Config File Opened";
  //   while (!f.eof()) {
  //     string SettingName;
  //     f >> SettingName;
  //     if (SettingName == "ClutchSet") {
  //       f >> ClutchSet_;
  //     } else if (SettingName == "BrakeSet") {
  //       f >> BrakeSet_;
  //     } else if (SettingName == "SpeedSet") {
  //       f >> SpeedSet_;
  //     } else if (SettingName == "ClutchReleaseRate") {
  //       f >> ClutchReleaseRate_;
  //     } else if (SettingName == "BrakeApplyRate") {
  //       f >> BrakeApplyRate_;
  //     } else if (SettingName == "IdleSpeed") {
  //       f >> IdleSpeed_;
  //     } else if (SettingName == "SpeedThreshold") {
  //       f >> SpeedThreshold_;
  //     } else if (SettingName == "SpeedErrorThreshold") {
  //       f >> SpeedErrorThreshold_;
  //     } else if (SettingName == "KSpeedThrottle") {
  //       f >> KSpeedThrottle_;
  //     } else if (SettingName == "KDrive") {
  //       f >> KDrive_;
  //     } else if (SettingName == "KBrake") {
  //       f >> KBrake_;
  //     }
  //   }
  //   f.close();
  // } else {
  //   AERROR << "ControlSettings.config Missing";
  // }

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

TransportController::~TransportController() {}

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
                                        const int AccEnable, 
                                        float vol_cur, float vol_exp) {
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
  }
  if (AccEnable == 1) {  //纵向控制启用
    int control_flag = 0;
    float ths_dif = SpeedErrorThreshold_;
    float ths_exp = SpeedThreshold_;

    if((vol_cur == 0) && (vol_exp > ths_exp)) {
      control_flag = 1;
    } else if ((vol_cur < ths_exp) || (vol_exp - vol_cur) > ths_dif) {
      control_flag = 2;
    } else if ((vol_exp - vol_cur) < ths_dif && (vol_cur > ths_exp)) {
      control_flag = 3;
    } else if (vol_exp < IdleSpeed_) {
      control_flag = 4;
    }
    
    if (control_flag) {
      switch(control_flag) {
        //start mode
        case 1:
        //brake set
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(0);
          // for(int i = clutchSet; i >= 0; i++) {
          for(int i = 100; i >= 0; i++) {
            //li he
            id_0xc040b2b_->set_xbr1_clupedalopenreq(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / transport_can_conf_.clutchreleaserate()));
          }
          break;
        //normal mode
        case 2:
          //P control
          if(vol_exp > vol_cur) {
            while(vol_exp > vol_cur) {
              vol_cur = vol_cur + (vol_exp - vol_cur) * transport_can_conf_.kdrive();
              id_0xc040b2b_->set_xbr1_accpedalopenreq(int(vol_cur / transport_can_conf_.kspeedthrottle()));
              std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }
          } else {
            while(vol_cur > vol_exp) {
              vol_cur = vol_cur - (vol_cur - vol_exp) * transport_can_conf_.kdrive();
              id_0xc040b2b_->set_xbr1_accpedalopenreq(int(vol_cur / transport_can_conf_.kspeedthrottle()));
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
          }
          break;
        //emergency mode
        case 3:
          //P control
          while(vol_cur > vol_exp) {
            vol_cur = vol_cur + (vol_exp - vol_cur) * transport_can_conf_.kbrake();
            id_0xc040b2b_->set_xbr1_brkpedalopenreq(int(vol_cur / transport_can_conf_.kspeedthrottle()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          break;
        //stop mode
        case 4:
          //li he
          // id_0xc040b2b_->set_xbr1_brkpedalopenreq(clutchSet);
          id_0xc040b2b_->set_xbr1_brkpedalopenreq(0);
          // for(int i = brakeSet; i <= 100; i++) {
          for(int i = 0; i <= 100; i++) {
            id_0xc040b2b_->set_xbr1_clupedalopenreq(i);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / transport_can_conf_.brakeapplyrate()));
          }
          break;
        default: break;
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
