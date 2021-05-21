#include "transport_canbus_component.h"

#include <cstdio>
#include <iostream>
#include <string>

namespace apollo {
namespace canbus {

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

  gps_reader_ = node_->CreateReader<Gps>(
      "/transport/gps",
      [this](const std::shared_ptr<Gps>& msg) { gps_.CopyFrom(*msg); });

  shared_cmd_reader_ = node_->CreateReader<ControlCommand>(
      "/transport/control", [this](const std::shared_ptr<ControlCommand>& msg) {
        vol_cur_ = gps_.gps_velocity() * 3.6;
        AINFO << "After read gps, vol_cur_ = " << vol_cur_;
        OnControl(*msg);
      });
  flag_reader_= node_->CreateReader<ControlFlag>(
      "/transport/controlflag", [this](const std::shared_ptr<ControlFlag>& msg) {
        UpdateFlag(*msg);
      });
  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("/transport/chassisdetail");
  AINFO << "Canbus Init";
  return true;
}

// Timer callback　API
bool transport_Canbus::Proc() {
  PublishChassisDetail();
  return true;
}

// shutdown
void transport_Canbus::Clear() {
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
  int latswitch=0;
  if(control_setting_conf_.latconswitch() == 1 && msg.control_steer_flag()==1)
      latswitch = 1;

  cmd.set_control_steer(msg.control_steer());  
  cmd.set_control_acc(msg.control_acc());
  cmd.set_control_accpedal(msg.control_accpedal());
  cmd.set_control_brkpedal(msg.control_brkpedal());
  cmd.set_control_clupedal(msg.control_clupedal());
  cmd.set_control_accpedal_flag(msg.control_accpedal_flag());
  cmd.set_control_brkpedal_flag(msg.control_brkpedal_flag());
  cmd.set_control_clupedal_flag(msg.control_clupedal_flag());

  transport_controller.ControlUpdate(cmd, latswitch,
                                     control_setting_conf_.lonconswitch(),
                                     vol_cur_, msg.control_acc());
  can_sender.Update();
  return;
}
void transport_Canbus::UpdateFlag(ControlFlag& msg) {
  static ControlFlag controlflag;
  controlflag.CopyFrom(msg);

  transport_controller.FlagUpdate(controlflag);
  can_sender.Update();
  return;
}


}  // namespace canbus
}  // namespace apollo
