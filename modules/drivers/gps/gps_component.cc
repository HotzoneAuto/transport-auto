#include "gps_component.h"

#include <cstdio>
#include <iostream>
#include <string>
using std::cout;
using std::endl;
using namespace std;

bool transport_Gps::Init() {
  // CAN1 Open
  CANCardParameter CanPara = CANCardParameter();
  CanClient_gps = std::unique_ptr<SocketCanClientRaw>(new SocketCanClientRaw());
  CanPara.set_type(CANCardParameter::PCI_CARD);
  CanPara.set_brand(CANCardParameter::SOCKET_CAN_RAW);
  CanPara.set_channel_id(CANCardParameter::CHANNEL_ID_ONE);
  CanPara.set_interface(CANCardParameter::NATIVE);
  CanClient_gps->Init(CanPara);
  CanClient_gps->Start();
  // Init can1 message_menager
  message_manager_gps =
      std::unique_ptr<MessageManager<Gps> >(new TransportGPSMessageManager);
  message_manager_gps->ClearSensorData();
  // CAN1 receiver
  can_receiver_gps.Init(CanClient_gps.get(), message_manager_gps.get(), 1);
  can_receiver_gps.Start();

  AINFO << "Transport_gps Init";

  chassis_detail_gps_writer_ = node_->CreateWriter<Gps>("/transport/gps");
  return true;
}

bool transport_Gps::Proc() {  // Timer callback
  PublishGps();
  return true;
}

void transport_Gps::PublishGps() {
  message_manager_gps->GetSensorData(&sensordata2);
  // AINFO << sensordata2.DebugString();
  double vel =
      sqrt(sensordata2.velocity_lateral() * sensordata2.velocity_lateral() +
           sensordata2.velocity_forward() * sensordata2.velocity_forward());
  sensordata2.set_gpsnh(sensordata2.gpsnh());
  sensordata2.set_gpsnl(sensordata2.gpsnl());
  sensordata2.set_gpseh(sensordata2.gpseh());
  sensordata2.set_gpsel(sensordata2.gpsel());
  chassis_detail_gps_writer_->Write(sensordata2);
}
