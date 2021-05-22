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
#include "cyber/time/time.h"

#include "modules/common/file/file.h"
#include "modules/canbus/proto/transport_can_conf.pb.h"
#include "modules/control/proto/control_command.pb.h"
#include "modules/control/proto/control_setting_conf.pb.h"
#include "modules/planning/proto/trajectory.pb.h"
#include "modules/drivers/gps/gps_protocol.h"
#include "modules/drivers/gps/proto/gps.pb.h"

using apollo::cyber::Time;
using apollo::canbus::TransportCanConf;
using apollo::control::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::planning::Trajectory;
using apollo::drivers::Gps;

class transport_Control : public apollo::cyber::Component<Trajectory> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Trajectory>& msg0) override;
  void Clear() override;

 private:
  std::shared_ptr<Writer<ControlCommand>> writer;
  ControlCommand controlcmd;
  double CaculateSteer(const std::shared_ptr<Trajectory>& msg0);
  double CaculateAcc(const std::shared_ptr<Trajectory>& msg0);

  double Stanley(double k, double v, int& ValidCheck);
  double LookAheadPredict();
  int FindLookahead(double totaldis);
  void CalculatePedalGear(double vol_exp, double delta_t);

  std::vector<double> rel_loc[7];

  apollo::control::ControlSettingConf control_setting_conf_;

  int control_brkpedal_flag = 0;
  int control_accpedal_flag = 0;
  int control_clupedal_flag = 0;
  double control_brkpedal = 0;
  double control_accpedal = 0;
  double control_clupedal = 0;
  // TODO: check gear
  int control_gear = 0;

  int control_flag = 0;
  int start_flag = 0;
  int finishstop_flag = 0;
  int wait_flag = 1;
  double wait_time = 0;
  int wait_count = 0;
  double cluopen_last = 0;
  double brkopen_last = 0;
  double delta_brk = 0;
  double delta_clu = 0;

  double vol_cur;
  double v_lateral;
  double v_forward;
  double heading_angle_now;
  double yaw_rate_now;

  uint64_t nanotime_last = 0;
  uint64_t nanotime_now = 0;
  uint64_t nanotime_init = 0;
  double delta_t = 0;

  // TODO: initialize kb and kf
  double kb[4] = {0.0975,0.0119,1.5071,0.0189};
  double kf[21] = {-0.0380,-0.0361,-0.0344,-0.0327,-0.0311,-0.0295,-0.0280,-0.0266,-0.0253,-0.0240,-0.0227,-0.0215,-0.0204,-0.0193,-0.0182,-0.0172,-0.0163,-0.0153,-0.0145,-0.0136,-0.0128};
  //record file
  apollo::common::file::File file_csv;
  std::string fname;
  int frame=0;

  Gps gps_;
  std::shared_ptr<apollo::cyber::Reader<Gps>> gps_reader_;
};
CYBER_REGISTER_COMPONENT(transport_Control)
