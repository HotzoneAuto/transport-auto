#include <iomanip>
#include <iostream>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
// transport
#include "modules/transport_can/vehicle//transport/transport_message_manager.h"
#include "modules/transport_can/vehicle/transport/transport_controller.h"
// transportgps
#include "modules/transport_can/vehicle//transportgps/transportgps_message_manager.h"

#define RecordMode 0
#define ControlMode 1
using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::canbus::Transport::TransportMessageManager;
using apollo::canbus::TransportGPS::TransportGPSMessageManager;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using ::apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;
using namespace std;
class transport_Canbus : public apollo::cyber::TimerComponent {
 public:
 private:
  int SteerEnable;
  int AccEnable;
  int Mode;
  ofstream TrajFile;
  bool Init() override;
  bool Proc() override;
  void Clear() override;
  void ReadConfig();
  void PublishChassisDetail();
  void OnControl(ControlCommand& msg);

  std::unique_ptr<SocketCanClientRaw> CanClient, CanClient_gps;
  std::unique_ptr<MessageManager<ChassisDetail> > message_manager,
      message_manager_gps;
  CanReceiver<ChassisDetail> can_receiver, can_receiver_gps;
  CanSender<ChassisDetail> can_sender;
  TransportController transport_controller;
  std::shared_ptr<apollo::cyber::Writer<ChassisDetail> > chassis_detail_writer_;
  std::shared_ptr<apollo::cyber::Reader<ControlCommand> >
      control_command_reader_;
};
CYBER_REGISTER_COMPONENT(transport_Canbus)