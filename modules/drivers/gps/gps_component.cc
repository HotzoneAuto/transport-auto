#include "gps_component.h"

#include <cstdio>
#include <iostream>
#include <string>

namespace apollo {
namespace drivers {
namespace gps {

bool transport_Gps::Init() {
  // CAN1 Open
  CANCardParameter CanPara = CANCardParameter();
  can_client_gps =
      std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ONE);
  CanPara.set_interface(CANCardParameter::NATIVE);
  can_client_gps->Init(CanPara);
  can_client_gps->Start();
  // Init can1 message_menager
  message_manager_gps =
      std::unique_ptr<MessageManager<Gps> >(new TransportGPSMessageManager);
  message_manager_gps->ClearSensorData();
  // CAN1 receiver
  can_receiver_gps.Init(can_client_gps.get(), message_manager_gps.get(), 1);
  can_receiver_gps.Start();

  AINFO << "Transport_gps Init";

  gps_writer_ = node_->CreateWriter<Gps>("/transport/gps");
  return true;
}

bool transport_Gps::Proc() {
  PublishGps();
  return true;
}

void transport_Gps::PublishGps() {
  message_manager_gps->GetSensorData(&gps_data);
  // double vel =
  //     std::sqrt(gps_data.velocity_lateral() * gps_data.velocity_lateral() +
  //          gps_data.velocity_forward() * gps_data.velocity_forward());

  gps_writer_->Write(gps_data);
}
}  // namespace gps
}  // namespace drivers
}  // namespace apollo
