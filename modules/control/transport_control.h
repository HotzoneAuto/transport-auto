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

#include "modules/control/proto/control_command.pb.h"
#include "modules/control/proto/control_setting_conf.pb.h"
#include "modules/drivers/gps/gps_protocol.h"
#include "modules/drivers/gps/proto/gps.pb.h"

using apollo::control::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Gps;

class transport_Control : public apollo::cyber::Component<Gps> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Gps>& msg0) override;
  void Clear() override;

 private:
  std::shared_ptr<Writer<ControlCommand>> writer;
  ControlCommand controlcmd;
  double Stanley(double k, double v, int& ValidCheck);
  double CaculateSteer(const std::shared_ptr<Gps>& msg0);
  double CaculateAcc(const std::shared_ptr<Gps>& msg0);
  void ReadTraj();
  void UpdateTraj(const std::shared_ptr<Gps>& msg0);
  int FindLookahead(double totaldis);

  std::vector<double> trajinfo[6];
  std::vector<double> rel_loc[3];

  apollo::control::ControlSettingConf control_setting_conf_;
  std::fstream TrajFile;
  std::fstream traj_record_file;
  int TrajIndex = 0;
  int frame = 0;

};
CYBER_REGISTER_COMPONENT(transport_Control)
