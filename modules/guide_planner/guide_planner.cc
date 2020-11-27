#include "guide_planner.h"
const float L=3.975;

bool guide_Planner::Init() {
  writer = node_->CreateWriter<TrajInfo>("guide/TrajInfo");
  AINFO << "TrajInfo Writer init";
  AINFO << "Guide_Planner init";
  return true;
}

bool guide_Planner::Proc(const std::shared_ptr<ChassisDetail>& msg0) {
  AINFO << "distance = " << msg0->uwb_distance();
  TrajUpdate(msg0);
  WriteTraj();
  return true;
}

void guide_Planner::TrajUpdate(const std::shared_ptr<ChassisDetail>& msg0) {
  static float last_vx_2;
  static float vx_2;
  static float last_yaw_rate_2;
  static float yaw_rate_2;

  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();
  float lat_distance = -distance * sin(azimuth_angle / 180 * M_PI);
  float long_distance = distance * cos(azimuth_angle / 180 * M_PI)+L;
  float vx_1 = msg0->leader_speed();

  float delta_f_2 = -(msg0->steer_angle() - 4.8505) / 24.1066;
  float yaw_rate_1 = 0;
  float rel_yaw = 0;  // todo:add leader yaw rate in algorithm
  if (!planner.isinit()) {
    if (distance > 1) {
      last_vx_2 = 0;
      last_yaw_rate_2 = 0;
      vx_2 = msg0->x_speed();
      yaw_rate_2 = msg0->follower_yaw_rate();
      planner.Init(long_distance, lat_distance, rel_yaw);
      AINFO << "Planner Inited, distance" << distance;
    } else
      return;
  } else {
    last_vx_2 = vx_2;
    vx_2 = msg0->x_speed();
    last_yaw_rate_2 = yaw_rate_2;
    yaw_rate_2 = msg0->follower_yaw_rate();
    AINFO<<"LongDistance: "<<long_distance;
    AINFO<<"lat_distance: "<<lat_distance;
    AINFO<<"last_vx_2: "<<last_vx_2;
    AINFO<<"last_yaw_rate_2: "<<last_yaw_rate_2;
    AINFO<<"delta_f_2: "<<delta_f_2;
    AINFO<<"TrajwillUpdate,size = "<<planner.traj.Seqv[0].size();
    planner.traj_seq_update(long_distance, lat_distance, vx_1, yaw_rate_1,
                            last_vx_2, last_yaw_rate_2, delta_f_2, rel_yaw);
    AINFO<<"TrajUpdated,size = "<<planner.traj.Seqv[0].size();
  }
  return;
}
void guide_Planner::WriteTraj() {
  if (planner.isinit()) {
    AINFO << "Write Traj";
    TrajInfo trajinfo;
    trajinfo.clear_rel_x();
    trajinfo.clear_rel_y();
    if(planner.traj.Seqv[0].size()>0)
        AINFO<<"traj sz: "<<planner.traj.Seqv[0].size();
    else
	AERROR<<"traj sz: "<<planner.traj.Seqv[0].size();
    for (int i = 0; i < planner.traj.Seqv[0].size(); i++) {
      trajinfo.add_rel_x(planner.traj.Seqv[0][i]);
      trajinfo.add_rel_y(planner.traj.Seqv[1][i]);
    }
    writer->Write(trajinfo);
    int sz = trajinfo.rel_x_size();
    if(sz>0)
    AINFO << "relx size: " << sz << " rel_x: " << trajinfo.rel_x(sz - 1);
    else 
    AERROR << "relx size: " << sz;
  }
}
