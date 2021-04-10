#pragma once

#include <iomanip>
#include <iostream>
#include <vector>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/canbus/transport_controller.h"
#include "modules/control/proto/control_setting_conf.pb.h"
#include "modules/planning/proto/planning_setting_conf.pb.h"
#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/gps/gps_protocol.h"
#include "modules/drivers/gps/proto/gps.pb.h"


namespace apollo {
namespace canbus {

using apollo::cyber::Reader;
using apollo::cyber::Writer;

using apollo::canbus::ChassisDetail;
using apollo::canbus::transport::TransportMessageManager;
using apollo::control::ControlCommand;
using apollo::drivers::Gps;

using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;

using apollo::cyber::common::GetProtoFromFile;
class transport_Canbus : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void PublishChassisDetail();
  void OnControl(ControlCommand& msg);

 private:
  int SteerEnable;
  int AccEnable;
  int Mode;
  int CurrentTrajNumber;
  double vol_cur_;
  Gps gps_;
  ChassisDetail sensordata;
  std::unique_ptr<SocketCanClientRaw> CanClient;
  std::unique_ptr<MessageManager<ChassisDetail>> message_manager;
  CanReceiver<ChassisDetail> can_receiver;
  CanSender<ChassisDetail> can_sender;
  TransportController transport_controller;
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail>> chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<Gps>> gps_reader_;
  std::shared_ptr<apollo::cyber::Reader<ControlCommand>> shared_cmd_reader_;
  apollo::control::ControlSettingConf control_setting_conf_;
  apollo::planning::PlanningSettingConf planning_setting_conf_;
};
CYBER_REGISTER_COMPONENT(transport_Canbus)
}  // namespace canbus
}  // namespace apollo
