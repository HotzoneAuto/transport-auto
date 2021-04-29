#pragma once

#include <cstdio>

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/timer/timer.h"
#include "modules/common/file/file.h"
#include "modules/common/time/time.h"
#include "modules/drivers/gps/gps_protocol.h"
#include "modules/drivers/gps/proto/gps.pb.h"
#include "modules/planning/proto/planning_setting_conf.pb.h"
#include "modules/planning/proto/trajectory.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/control/proto/control_flag.pb.h"

#define TRAJPLOTLENGTH 400
#define TRAJFINDLENGTH 100
#define MAXDIS 99999
#define L 2.4

using apollo::cyber::Component;
using apollo::drivers::Gps;
using apollo::canbus::ChassisDetail;
using apollo::control::ControlFlag;

class TransportPlanning : public apollo::cyber::Component<Gps> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Gps>& msg0) override;
  void Clear() override;

 private:
  void ReadTraj();
  void UpdateTraj(const std::shared_ptr<Gps>& msg0);
  bool ChangeTraj(int TrajNumber);
  std::vector<double> trajinfo[6];

  apollo::common::file::File file_csv;
  std::fstream traj_record_file;
  std::fstream traj_draw_file;
  int TrajIndex = 0;
  int frame = 0;
  int CurrentTrajNumber=1;
  ControlFlag controlflag_;
  std::string fname = "/apollo/modules/planning/data/gps_record.csv";
  apollo::planning::PlanningSettingConf planning_setting_conf_;
  std::shared_ptr<apollo::cyber::Writer<apollo::planning::Trajectory>>
      trajs_writer = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlFlag>>
      controlflag_reader_= nullptr;
  std::shared_ptr<apollo::planning::Trajectory> msg_traj;
};
CYBER_REGISTER_COMPONENT(TransportPlanning)
