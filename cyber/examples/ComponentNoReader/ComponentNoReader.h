/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <memory>
#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "cyber/examples/proto/examples.pb.h"
#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::examples::proto::Chatter;
using apollo::drivers::canbus::can::SocketCanClientRaw;
using apollo::drivers::canbus::CANCardParameter;
class ComponentNoReader : public Component<>{
 public:
  bool Init() override;
 private:
 std::unique_ptr<apollo::cyber::Node> listener_node ;
 //std::unique_ptr<SocketCanClientRaw>  SocClient;

};
CYBER_REGISTER_COMPONENT(ComponentNoReader)
