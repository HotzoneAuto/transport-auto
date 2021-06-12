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
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_command.pb.h"
#include "modules/control/proto/control_flag.pb.h"
#include "modules/control/proto/control_toremote.pb.h"
#include "modules/control/proto/control_setting_conf.pb.h"
#include "modules/planning/proto/trajectory.pb.h"
#include "modules/planning/proto/planning_setting_conf.pb.h"
#include "modules/drivers/gps/gps_protocol.h"
#include "modules/drivers/gps/proto/gps.pb.h"


using apollo::cyber::Time;
using apollo::canbus::TransportCanConf;
using apollo::control::ControlCommand;
using apollo::control::ControlFlag;
using apollo::control::ControlToRemote;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::planning::Trajectory;
using apollo::drivers::Gps;
using apollo::canbus::ChassisDetail;
using apollo::cyber::common::GetProtoFromFile;

class transport_Control : public apollo::cyber::Component<Trajectory,ChassisDetail> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Trajectory>& msg0,
            const std::shared_ptr<ChassisDetail>& msg1) override;
  void Clear() override;

 private:
  std::shared_ptr<Writer<ControlCommand>> writer;
  std::shared_ptr<Writer<ControlFlag>> flag_writer;
  std::shared_ptr<Writer<ControlToRemote>> remote_writer;
  ControlCommand controlcmd;
  ControlFlag controlflag;
  ControlToRemote controltoremote;
  double CaculateSteer(const std::shared_ptr<Trajectory>& msg0);
  double CaculateAcc(const std::shared_ptr<Trajectory>& msg0);

  double Stanley(double k, double v, int& ValidCheck);
  int FindLookahead(double totaldis);
  void CalculatePedalGear(double vol_exp, double delta_t,Trajectory msg0,ChassisDetail msg1);

  std::vector<double> rel_loc[5];

  apollo::control::ControlSettingConf control_setting_conf_;
  apollo::planning::PlanningSettingConf planning_setting_conf_;

  int control_brkpedal_flag = 0;
  int control_accpedal_flag = 0;
  int control_clupedal_flag = 0;
  double control_brkpedal = 0;
  double control_accpedal = 0;
  double control_clupedal = 0;
  // TODO: check gear
  int control_gear = 0;

  double e_y = 0;
  double e_phi = 0;

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

  uint64_t nanotime_last = 0;
  uint64_t nanotime_now = 0;
  uint64_t nanotime_init = 0;
  double delta_t = 0;

  std::vector<double> accelset;
  std::vector<double> clutchreleaserate;
  std::vector<double> brakeapplyrate;
  std::vector<double> kspeedthrottle;
  std::vector<double> kdrive;
  std::vector<double> kbrake;

  Gps gps_;
  std::shared_ptr<apollo::cyber::Reader<Gps>> gps_reader_;

  apollo::common::file::File file_csv;
  std::string fname;
  int frame = 0;
};
CYBER_REGISTER_COMPONENT(transport_Control)
