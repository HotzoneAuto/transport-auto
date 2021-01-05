#include "transport_canbus_component.h"

#include <cstdio>
#include <iostream>
#include <string>

bool transport_Canbus::Init() {
  // read config;
  if (!GetProtoConfig(&control_setting_conf_)) {
    AERROR << "Unable to load conf file" << ConfigFilePath();
    return false;
  }
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
  // if (control_setting_conf_.trajmode() == RecordMode) {
  //   TrajFile.open("/apollo/modules/transportTraj.record");
  //   if (TrajFile.is_open()) AINFO << "TrajFileOpened";
  // } else if (control_setting_conf_.trajmode() == ControlMode) {
  // }
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
  chassis_detail_writer_->Write(sensordata);
  return;
}

// control callback function  will move to reader callback
void transport_Canbus::OnControl(ControlCommand& msg) {
  static ControlCommand cmd;
  // Write Control Here
  cmd.set_control_steer(msg.control_steer());
  // cmd.set_control_acc(msg.control_acc());
  transport_controller.ControlUpdate(cmd, control_setting_conf_.latconswitch(),
                                     control_setting_conf_.lonconswitch(), vol_cur_,
                                     msg.control_acc());
  can_sender.Update();
  return;
}

// TODO(ZENGPENG):CONFIG

