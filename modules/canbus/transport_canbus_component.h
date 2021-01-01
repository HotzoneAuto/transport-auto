#pragma once

#include <iomanip>
#include <iostream>
#include <vector>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/gps/GPSproto.h"
#include "modules/drivers/gps/proto/gps.pb.h"
#include "modules/canbus/transport_controller.h"

// TODO(ZENGPENG): SPLIT DATA RECORD FROM CHASSIS  
#define RecordMode 0
#define ControlMode 1

using apollo::cyber::Reader;
using apollo::cyber::Writer;

using apollo::canbus::ChassisDetail;
using apollo::control::ControlCommand;
using apollo::drivers::Gps;
using apollo::canbus::transport::TransportMessageManager;

using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;

class transport_Canbus : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void ReadConfig();
  void PublishChassisDetail();
  void OnControl(ControlCommand& msg);

 private:
  int SteerEnable;
  int AccEnable;
  int Mode;
  std::ofstream TrajFile;
  double vol_cur_;
  double vol_exp_;
  Gps gps_;
  ChassisDetail sensordata;
  std::unique_ptr<SocketCanClientRaw> CanClient;
  std::unique_ptr<MessageManager<ChassisDetail>> message_manager;
  CanReceiver<ChassisDetail> can_receiver;
  CanSender<ChassisDetail> can_sender;
  TransportController transport_controller;
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail>> chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<Gps>>
      gps_reader_;
  std::shared_ptr<apollo::cyber::Reader<ControlCommand>>
      control_command_reader_;
};
CYBER_REGISTER_COMPONENT(transport_Canbus)
