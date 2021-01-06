#pragma once

#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"

#include "modules/canbus/proto/transport_can_conf.pb.h"
#include "modules/control/proto/control_command.pb.h"
#include "modules/control/proto/control_setting_conf.pb.h"
#include "modules/planning/proto/trajectory.pb.h"

using apollo::canbus::TransportCanConf;
using apollo::control::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;

class transport_Control : public apollo::cyber::Component<apollo::planning::Trajectory> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<apollo::planning::Trajectory>& msg0) override;

 private:
  std::shared_ptr<Writer<ControlCommand>> writer;
  ControlCommand controlcmd;
  double CaculateSteer(const std::shared_ptr<apollo::planning::Trajectory>& msg0);
  double CaculateAcc(const std::shared_ptr<apollo::planning::Trajectory>& msg0);
  double Stanley(double k, double v, int& ValidCheck);
  int FindLookahead(double totaldis);

  std::vector<double> rel_loc[4];

  apollo::control::ControlSettingConf control_setting_conf_;
};
CYBER_REGISTER_COMPONENT(transport_Control)
