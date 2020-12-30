#include "transport_canbus_component.h"

#include <cstdio>
#include <iostream>
#include <string>

bool transport_Canbus::Init() {
  // read config;
  ReadConfig();
  // Open CAN0 
  CANCardParameter CanPara = CANCardParameter();

  CanClient = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  // TODO(ZENGPENG):CONFIG CAN INIT PARAMETERS
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient->Init(CanPara);
  CanClient->Start();
  
  // Init can0 message_menager
  message_manager = std::unique_ptr<MessageManager<ChassisDetail> >(
      new TransportMessageManager);
  message_manager->ClearSensorData();
  
  // CAN0 receiver&sender
  can_receiver.Init(CanClient.get(), message_manager.get(), 1);
  can_receiver.Start();
  can_sender.Init(CanClient.get(), 1);
  can_sender.Start();
  
  // Init CAN0 Controller
  transport_controller.Init(&can_sender, message_manager.get());
  transport_controller.Start();
  
  AINFO << "Canbus Init";

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("/transport/chassisdetail");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "/transport/control",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });

  // TODO(ZENGPENG): SPLIT DATA RECORD FROM CHASSIS 
  if (Mode == RecordMode) {
    TrajFile.open("/apollo/modules/transportTraj.record");
    if (TrajFile.is_open()) AINFO << "TrajFileOpened";
  } else if (Mode == ControlMode) {
  }
  return true;
}

// Timer callback　API
bool transport_Canbus::Proc() {
  PublishChassisDetail();
  return true;
}

// shutdown
void transport_Canbus::Clear() {
  if (TrajFile.is_open()) {
    AINFO << "TrajFileClosed";
    TrajFile.close();
  }
  transport_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  CanClient->Stop();
}

void transport_Canbus::PublishChassisDetail() {
  message_manager->GetSensorData(&sensordata);
  sensordata.set_current_steer_angle(sensordata.current_steer_angle());
  chassis_detail_writer_->Write(sensordata);
  return;
}

// control callback function  will move to reader callback
void transport_Canbus::OnControl(ControlCommand& msg) {
  static ControlCommand cmd;
  // Write Control Here
  cmd.set_control_steer(msg.control_steer());
  // cmd.set_control_acc(msg.control_acc());
  transport_controller.ControlUpdate(cmd, SteerEnable, AccEnable, vol_cur_,
                                     msg.control_acc());
  can_sender.Update();
  return;
}

// TODO(ZENGPENG):CONFIG
void transport_Canbus::ReadConfig() {
  ifstream f;
  f.open("/apollo/modules/control/conf/ControlSettings.config");
  if (f.is_open()) {
    AINFO << "Control Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      if (SettingName == "LonConSwitch") {
        f >> AccEnable;
        AINFO << "AccEnable= " << AccEnable;
      } else if (SettingName == "LatConSwitch") {
        f >> SteerEnable;
        AINFO << "SteerEnable= " << SteerEnable;
      } else if (SettingName == "TrajMode") {
        f >> Mode;
        AINFO << "TrajMode= " << Mode;
      }
    }
    f.close();
  } else
    AERROR << "ControlSettings.config Missing";
}
