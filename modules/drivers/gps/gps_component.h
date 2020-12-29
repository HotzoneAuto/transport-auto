#include <iomanip>
#include <iostream>
#include <vector>
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"

#include "modules/drivers/gps/message_manager.h"

using apollo::canbus::TransportGPS::TransportGPSMessageManager;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Gps;
using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;

class transport_Gps : public apollo::cyber::TimerComponent {
 public:
 private:
  int vol_cur_;
  bool Init() override;
  bool Proc() override;
  void PublishGps();

  Gps sensordata2;
  std::unique_ptr<SocketCanClientRaw> CanClient_gps;
  std::unique_ptr<MessageManager<Gps>> message_manager_gps;
  CanReceiver<Gps> can_receiver_gps;
  std::shared_ptr<apollo::cyber::Writer<Gps>> chassis_detail_gps_writer_;
};
CYBER_REGISTER_COMPONENT(transport_Gps)