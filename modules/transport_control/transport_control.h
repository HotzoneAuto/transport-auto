#include <iostream>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"

#include "modules/transport_can/proto/chassis_detail.pb.h"
#include "modules/transport_can/proto/control_command.pb.h"
#include "modules/transport_control/gps/GPSproto.h"

using apollo::canbus::ChassisDetail;
using apollo::canbus::ControlCommand;
using apollo::cyber::Component;
using apollo::cyber::ComponentBase;
using apollo::cyber::Writer;
class transport_Control : public apollo::cyber::Component<ChassisDetail> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ChassisDetail>& msg0) override;
  
 private:
  std::shared_ptr<Writer<ControlCommand> > writer;
  ControlCommand controlcmd;
  double Stanley(double k,double v,int &ValidCheck);
  double Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0);
  double Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0);
  void ReadTraj();
  void UpdateTraj(const std::shared_ptr<ChassisDetail>& msg0);
  int FindLookahead(double totaldis);
  vector<double> trajinfo[6];
  vector<double> rel_loc[3];

  fstream TrajFile;
  int TrajIndex;
  struct ConfigInfo{
    double LookAheadDis;
    double StanleyK;
  }configinfo;
};
CYBER_REGISTER_COMPONENT(transport_Control)
