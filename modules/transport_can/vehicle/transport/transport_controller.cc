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
                                        const int AccEnable) {
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
    // double vol_exp=0;
    // double vol_cur=0;//todo 后续有期望速度与踏板开度的标定
    // static control_flag=1;
    // if(vol_exp < 0){
    //   control_flag=4;
    // }

    // switch(control_flag) {
    // //start mode
    // case 1:
    // //zhi dong
    //   id_0x0c040b2a_->set_xbr1_brkpedalopenreq(0);
    //   for(int i = 0; i < 100; i++) {
    //     //li he
    //     id_0x0c040b2a_->set_xbr1_clupedalopenreq(i);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     if(i < int(vol_exp)){
    //       //you men
    //       id_0x0c040b2a_->set_xbr1_accpedalopenreq(i);
    //       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //   }
    //   control_flag=2;//状态切换
    //   break;
    // //normal mode
    // case 2:
    //   if(vol_exp > vol_cur) {
    //     for (int i = int(vol_cur); i < int(vol_exp); i++) {
    //       id_0x0c040b2a_->set_xbr1_accpedalopenreq(i);
    //       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //   } else {
    //     for (int i = int(vol_cur); i > int(vol_exp); i++) {
    //       id_0x0c040b2a_->set_xbr1_accpedalopenreq(i);
    //       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //   }
    //   break;
    // //emergency mode
    // case 3:
    //   for(int i = 0; i < 100; i++) {
    //     //li he
    //     id_0x0c040b2a_->set_xbr1_brkpedalopenreq(i);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //   }
    //   break;
    // //stop mode
    // case 4:
    //   id_0x0c040b2a_->set_xbr1_clupedalopenreq(100);
    //   id_0x0c040b2a_->set_xbr1_accpedalopenreq(0);
    //   for(int i = 0; i < 100; i++) {
    //     //li he
    //     id_0x0c040b2a_->set_xbr1_brkpedalopenreq(i);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //   }
    //   break;
    //   default: break;
    // }
  }
}
