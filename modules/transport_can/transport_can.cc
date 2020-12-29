#include "transport_can.h"

#include <cstdio>
#include <iostream>
#include <string>
using std::cout;
using std::endl;
using namespace std;

bool transport_Canbus::Init() {
  // read config;
  ReadConfig();
  // CAN0 Open
  CANCardParameter CanPara = CANCardParameter();

  CanClient = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
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
  transport_controller.Init(&can_sender, message_manager.get());
  transport_controller.Start();
  // Init CAN0 Controller

  // CAN1 Open
  CanClient_gps = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ONE);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient_gps->Init(CanPara);
  CanClient_gps->Start();
  // Init can1 message_menager
  message_manager_gps = std::unique_ptr<MessageManager<ChassisDetail> >(
      new TransportGPSMessageManager);
  message_manager_gps->ClearSensorData();
  // CAN1 receiver
  can_receiver_gps.Init(CanClient_gps.get(), message_manager_gps.get(), 1);
  can_receiver_gps.Start();

  AINFO << "Canbus Init";

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("/transport/chassisdetail");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "/transport/control",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });

  if (Mode == RecordMode) {
    TrajFile.open("/apollo/modules/transportTraj.record");
    if (TrajFile.is_open()) AINFO << "TrajFileOpened";
  } else if (Mode == ControlMode) {
  }
  return true;
}

bool transport_Canbus::Proc() {  // Timer callback
  PublishChassisDetail();
  return true;
}
void transport_Canbus::Clear() {  // shutdown
  if (TrajFile.is_open()) {
    AINFO << "TrajFileClosed";
    TrajFile.close();
  }
  transport_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  CanClient->Stop();

  can_receiver_gps.Stop();
  CanClient_gps->Stop();
  // std::cout<<"stopping and clearing"<<std::endl;
}
void transport_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata1, sensordata2;
  message_manager->GetSensorData(&sensordata1);
  message_manager_gps->GetSensorData(&sensordata2);
  sensordata2.set_current_steer_angle(sensordata1.current_steer_angle());
  AINFO << sensordata2.DebugString();
  if (Mode == RecordMode) {
    double vel =
        sqrt(sensordata2.velocity_lateral() * sensordata2.velocity_lateral() +
             sensordata2.velocity_forward() * sensordata2.velocity_forward());
    vol_cur_ = vel;
    // WriteTraj
    if (sensordata2.gpsnh() != 0) {
      TrajFile << setprecision(3) << sensordata2.gpsnh() << " ";
      TrajFile << setprecision(8) << sensordata2.gpsnl() << " ";
      TrajFile << setprecision(3) << sensordata2.gpseh() << " ";
      TrajFile << setprecision(8) << sensordata2.gpsel() << " ";
      TrajFile << setprecision(5) << vel << endl;
    }
  } else if (Mode == ControlMode) {
    chassis_detail_writer_->Write(sensordata2);
  }
  return;
}
void transport_Canbus::OnControl(ControlCommand& msg) {
  // control callback function  will move to reader callback
  static ControlCommand cmd;
  // Write Control Here
  cmd.set_control_steer(msg.control_steer());
  // cmd.set_control_acc(msg.control_acc());
  transport_controller.ControlUpdate(cmd, SteerEnable, AccEnable, vol_cur_,
                                     msg.control_acc());
  can_sender.Update();
  return;
}

void transport_Canbus::ReadConfig() {
  ifstream f;
  f.open("/apollo/modules/transport_control/ControlSettings.config");
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
