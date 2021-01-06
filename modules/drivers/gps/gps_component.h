#pragma once

#include <iomanip>
#include <iostream>
#include <vector>

#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/time/time.h"

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"

#include "modules/drivers/gps/message_manager.h"

namespace apollo {
namespace drivers {
namespace gps {

using apollo::cyber::Time;
using apollo::cyber::Writer;
using apollo::drivers::Gps;
using apollo::drivers::canbus::CANCardParameter;
using apollo::drivers::canbus::CanReceiver;
using apollo::drivers::canbus::CanSender;
using apollo::drivers::canbus::MessageManager;
using apollo::drivers::canbus::can::SocketCanClientRaw;
using apollo::drivers::gps::TransportGPSMessageManager;

class transport_Gps : public apollo::cyber::TimerComponent {
 public:
  bool Init() override;
  bool Proc() override;
  void PublishGps();

 private:
  int vol_cur_;
  Gps gps_data;
  std::unique_ptr<SocketCanClientRaw> can_client_gps;
  std::unique_ptr<MessageManager<Gps>> message_manager_gps;
  CanReceiver<Gps> can_receiver_gps;
  std::shared_ptr<apollo::cyber::Writer<Gps>> gps_writer_;
};
CYBER_REGISTER_COMPONENT(transport_Gps)

}  // namespace gps
}  // namespace drivers
}  // namespace apollo
