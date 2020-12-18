#include "transport_can.h"

#include <cstdio>
#include <iostream>
#include <string>
using std::cout;
using std::endl;
using namespace std;

bool transport_Canbus::Init() {
  // CAN0 Open
  CanClient = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CANCardParameter CanPara = CANCardParameter();
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient->Init(CanPara);
  CanClient->Start();
  // CAN1 Open
  CanClient_gps = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ONE);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient_gps->Init(CanPara);
  CanClient_gps->Start();

  // Init can0 message_menager
  message_manager = std::unique_ptr<MessageManager<ChassisDetail> >(
      new TransportMessageManager);
  message_manager->ClearSensorData();

  // Init can1 message_menager
  message_manager_gps = std::unique_ptr<MessageManager<ChassisDetail> >(
      new TransportGPSMessageManager);
  message_manager->ClearSensorData();

  // CAN0 receiver&sender
  can_receiver.Init(CanClient.get(), message_manager.get(), 1);
  can_receiver.Start();
  can_sender.Init(CanClient.get(), 1);
  can_sender.Start();

  // CAN1 receiver
  can_receiver_gps.Init(CanClient_gps.get(), message_manager_gps.get(), 1);
  can_receiver_gps.Start();

  // Init receiver sender

  transport_controller.Init(&can_sender, message_manager.get());
  transport_controller.Start();
  // Init CAN0 Controller
  AINFO << "Canbus Init";

  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("transport/ChassisDetailOrig");
  // Create Writer

  control_command_reader_ = node_->CreateReader<ControlCommand>(
      "transport/ControlCommand",
      [this](const std::shared_ptr<ControlCommand>& msg) { OnControl(*msg); });

  // read config;
  ReadConfig();
  if (Mode == RecordMode) {
    TrajFile.open("/apollo/modules/TrajFile.dat", ios::out);
  } else if (Mode == ControlMode) {
  }
  return true;
}

bool transport_Canbus::Proc() {  // Timer callback
  PublishChassisDetail();
  return true;
}
void transport_Canbus::Clear() {  // shutdown
  if (TrajFile.is_open()) TrajFile.close();
  transport_controller.Stop();
  can_sender.Stop();
  can_receiver.Stop();
  can_receiver_gps.Stop();
  CanClient->Stop();
  CanClient_gps->Stop();
  // std::cout<<"stopping and clearing"<<std::endl;
}
void transport_Canbus::PublishChassisDetail() {
  ChassisDetail sensordata1, sensordata2;
  message_manager->GetSensorData(&sensordata1);
  message_manager_gps->GetSensorData(&sensordata2);
  sensordata2.set_current_steer_angle(sensordata1.current_steer_angle());
  AINFO << sensordata2.DebugString();
  chassis_detail_writer_->Write(sensordata2);
  // WriteTraj
  TrajFile << setprecision(3) << sensordata2.gpsnh() << " ";
  TrajFile << setprecision(8) << sensordata2.gpsnl() << " ";
  TrajFile << setprecision(3) << sensordata2.gpseh() << " ";
  TrajFile << setprecision(8) << sensordata2.gpsel() << " ";
  TrajFile << setprecision(5) << sensordata2.gps_velocity() << endl;
  return;
}
void transport_Canbus::OnControl(
    ControlCommand&
        msg) {  // control callback function  will move to reader callback
  static ControlCommand cmd;
  // Write Control Here
  // cmd.set_control_steer(msg.control_steer());
  // cmd.set_control_acc(msg.control_acc());
  // transport_controller.ControlUpdate(cmd, SteerEnable, AccEnable);
  can_sender.Update();
  return;
}

void transport_Canbus::ReadConfig() {
  ifstream f;
  // f.open("/apollo/modules/transport_control/ControlSettings.config");
  // if (f.is_open()) {
  //   AINFO << "Control Config File Opened";
  //   while (!f.eof()) {
  //     string SettingName;
  //     f >> SettingName;
  //     if (SettingName == "LonConSwitch") {
  //       f >> AccEnable;
  //       AINFO << "AccEnable= " << AccEnable;
  //     } else if (SettingName == "LatConSwitch") {
  //       f >> SteerEnable;
  //       AINFO << "SteerEnable= " << SteerEnable;
  //     }
  //   }
  //   f.close();
  // } else
  //   AERROR << "ControlSettings.config Missing";

  f.open("/apollo/modules/transport_can/ModeSettings.config");
  if (f.is_open()) {
    AINFO << "Mode Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      if (SettingName == "Mode") {
        f >> Mode;
        AINFO << "Mode= " << Mode;
      }
    }
    f.close();
  } else
    AERROR << "ModeSettings.config Missing";
}