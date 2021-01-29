#include "transport_planning.h"

#include "modules/common/util/util.h"

bool TransportPlanning::Init() {
  AINFO << "Transport_Control init";
  if (!GetProtoConfig(&planning_setting_conf_)) {
    AERROR << "Unable to load conf file" << ConfigFilePath();
    return false;
  }
  AINFO << planning_setting_conf_.trajmode();
  AINFO << planning_setting_conf_.trajnumber();
  if( planning_setting_conf_.trajnumber() == 1){
    fname = "/apollo/modules/planning/data/gps_record1.csv";
  }else if(planning_setting_conf_.trajnumber() == 2 ){
    fname = "/apollo/modules/planning/data/gps_record2.csv";
  }
  if (!planning_setting_conf_.trajmode()) {
    file_csv.open_file(fname);
    std::string msg_w =
        "frame,pgsnh,gpsnl,gpseh,gpsel,heading_angle,"
        "yaw_rate,gps_state,gps_velocity,acceleration_forward,"
        "acceleration_lateral,acceleration_down,pitch_angle,"
        "velocity_down,velocity_lateral,velocity_forward,roll_angle,"
        "timestamp";
    file_csv.write_file(msg_w);
  } else {
    trajs_writer = node_->CreateWriter<apollo::planning::Trajectory>(
        "/transport/planning");
    ReadTraj();
  }
  return true;
}

void TransportPlanning::ReadTraj() {
  //读取所有点的经纬度
  traj_record_file.open(fname, std::ios::in);
  char linestr[500] = {0};
  traj_record_file.getline(linestr, 500);
  double last_gpsn=0,last_gpse=0;
  while (traj_record_file.getline(linestr, 500)) {
    std::stringstream ss(linestr);
    std::string csvdata[17];
    double gpsnh, gpsnl, gpseh, gpsel, velocity;
    for (int i = 0; i < 17; i++) {
      char tempdata[500] = {0};
      ss.getline(tempdata, 500, ',');
      csvdata[i] = std::string(tempdata);
    }
    gpsnh = atof(csvdata[1].data());
    gpsnl = atof(csvdata[2].data());
    gpseh = atof(csvdata[3].data());
    gpsel = atof(csvdata[4].data());
    velocity = atof(csvdata[8].data());
    if( last_gpsn==0 || apollo::drivers::gps::SphereDis(
          last_gpse,last_gpsn, gpseh+gpsel, gpsnh+gpsnl) > 0.1){
      trajinfo[0].push_back(gpsnh);
      trajinfo[1].push_back(gpsnl);
      trajinfo[2].push_back(gpseh);
      trajinfo[3].push_back(gpsel);
      trajinfo[4].push_back(velocity);
      last_gpsn=gpsnh+gpsnl;
      last_gpse=gpseh+gpsel;
    }
  }
  traj_record_file.close();
}

void TransportPlanning::UpdateTraj(const std::shared_ptr<Gps>& msg0) {
  //将靠近的若干个点转移到车辆的坐标系中
  //寻找轨迹上一段范围内的距离本车最近的点
  double N_now = msg0->gpsnl() + msg0->gpsnh();
  double E_now = msg0->gpsel() + msg0->gpseh();
  double Azi_now = msg0->heading_angle() / 180 * M_PI;
  int lastindex = TrajIndex;
  double min_dis = MAXDIS;
  for (int i = lastindex;
       i < std::min(lastindex + TRAJLENGTH / 2, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[2][i] + trajinfo[3][i];
    double dis =
        apollo::drivers::gps::SphereDis(E_now, N_now, E_point, N_point);
    double azi =
        apollo::drivers::gps::SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * std::cos(azi - Azi_now);
    double rel_y = dis * std::sin(azi - Azi_now);
    if (std::abs(rel_x) < min_dis) {
      min_dis = std::abs(rel_x);
      TrajIndex = i;
    }
  }
  AINFO << "TrajIndex= " << TrajIndex << "  MINDIS=" << min_dis;
  //将该点附近的若干个点加入到自车坐标系中
  for (int i = TrajIndex;
       i < std::min(TrajIndex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[2][i] + trajinfo[3][i];
    double dis =
        apollo::drivers::gps::SphereDis(E_now, N_now, E_point, N_point);
    double azi =
        apollo::drivers::gps::SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * std::cos(azi - Azi_now);
    double rel_y = dis * std::sin(azi - Azi_now);
    double vel = trajinfo[4][i];
    auto* traj_position = msg_traj->add_points();
    traj_position->set_rel_x(rel_x);
    traj_position->set_rel_y(rel_y);
    traj_position->set_rel_vel(vel);
    traj_position->set_timestamp(apollo::cyber::Time::Now().ToNanosecond());
  }
  //记录轨迹用于绘图
  traj_draw_file.open("/apollo/modules/traj.record",std::ios::out);
  traj_draw_file << msg_traj->points_size() << std::endl;
  for(int i=0;i< msg_traj->points_size();i++){
    traj_draw_file <<  msg_traj->points(i).rel_x() << " " 
        <<msg_traj->points(i).rel_y() << std::endl;
  }
  traj_draw_file.close();
  // msg_traj->mutable_header()->set_timestamp_sec(apollo::common::time::Clock::NowInSeconds());
  // trajs_writer->Write(msg_traj);
  double control_acc = 0;
  int length = trajinfo[0].size();
  double DisToStart = 0;
  for(int i=1;i<=TrajIndex;i++){
      double N_1 = trajinfo[0][i-1] + trajinfo[1][i-1];
      double E_1 = trajinfo[2][i-1] + trajinfo[3][i-1];
      double N_2 = trajinfo[0][i] + trajinfo[1][i];
      double E_2 = trajinfo[2][i] + trajinfo[3][i];
      DisToStart = DisToStart +
          apollo::drivers::gps::SphereDis(E_1, N_1, E_2, N_2);
  }
  double DisToEnd = 0;
  for(int i = TrajIndex + 1;i < length;i++){
    double N_1 = trajinfo[0][i-1] + trajinfo[1][i-1];
    double E_1 = trajinfo[2][i-1] + trajinfo[3][i-1];
    double N_2 = trajinfo[0][i] + trajinfo[1][i];
    double E_2 = trajinfo[2][i] + trajinfo[3][i];
    DisToEnd = DisToEnd +
        apollo::drivers::gps::SphereDis(E_1, N_1, E_2, N_2);
  }
  
  // control_acc = std::min(DisToStart, DisToEnd) *
  // control_setting_conf_.speedk(); control_acc =
  // std::min(control_setting_conf_.desiredspeed(), control_acc);

  // Traj speed mode
  control_acc = trajinfo[4][TrajIndex];
  msg_traj->set_dis_to_start(DisToStart);
  msg_traj->set_dis_to_end(DisToEnd);
  msg_traj->set_control_acc(control_acc);
  msg_traj->set_gps_state(msg0->gps_state());
}

/*
  Reader Callback function
*/
bool TransportPlanning::Proc(const std::shared_ptr<Gps>& msg0) {
  if (!planning_setting_conf_.trajmode()) {
    if (frame == 65535) {
      frame = 0;
    }
    frame++;

    std::string msg_w = std::to_string(frame) + "," + std::to_string(msg0->gpsnh()) 
                    + "," + std::to_string(msg0->gpsnl()) 
                    + "," + std::to_string(msg0->gpseh()) 
                    + "," + std::to_string(msg0->gpsel()) 
                    + "," + std::to_string(msg0->heading_angle()) 
                    + "," + std::to_string(msg0->yaw_rate()) 
                    + "," + std::to_string(msg0->gps_state()) 
                    + "," + std::to_string(msg0->gps_velocity())
                    + "," + std::to_string(msg0->acceleration_forward()) 
                    + "," + std::to_string(msg0->acceleration_lateral()) 
                    + "," + std::to_string(msg0->acceleration_down()) 
                    + "," + std::to_string(msg0->pitch_angle()) 
                    + "," + std::to_string(msg0->velocity_down()) 
                    + "," + std::to_string(msg0->velocity_lateral()) 
                    + "," + std::to_string(msg0->velocity_forward()) 
                    + "," + std::to_string(msg0->roll_angle())
                    + "," + std::to_string(msg0->timestamp());
    file_csv.write_file(msg_w);
  } else {
    msg_traj.reset();
    msg_traj = std::make_shared<apollo::planning::Trajectory>();
    UpdateTraj(msg0);
    // CaculateAcc(msg0);
    trajs_writer->Write(msg_traj);
  }
  return true;
}

void TransportPlanning::Clear() {
  if (!planning_setting_conf_.trajmode()) {
    file_csv.close_file();
  }
}
