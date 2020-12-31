#pragma once

#include <cstdio>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "cyber/cyber.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "modules/drivers/gps/GPSproto.h"
#include "modules/drivers/gps/proto/gps.pb.h"
#include "modules/common/file/file.h"
#include "modules/planning/proto/planning_setting_conf.pb.h"
#include "modules/planning/proto/rel_loc.pb.h"

#define TRAJLENGTH 200
#define MAXDIS 99999
#define L 2.4

using apollo::drivers::Gps;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;

class transport_Planning : public apollo::cyber::Component<Gps> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Gps>& msg0) override;
  void Clear() override;

 private:
  void ReadTraj();
  void UpdateTraj(const std::shared_ptr<Gps>& msg0);

  std::vector<double> trajinfo[6];
  std::vector<double> rel_loc[3];
  
  apollo::common::file::File file_csv;
  std::fstream traj_record_file;
  int TrajIndex;
  int frame;
  std::string fname = "/apollo/modules/planning/data/gps_record.csv";
  apollo::planning::PlanningSettingConf planning_setting_conf_;
  std::shared_ptr<apollo::cyber::Writer<apollo::planning::RelLoc>> rel_loc_writer;
};
CYBER_REGISTER_COMPONENT(transport_Planning)
