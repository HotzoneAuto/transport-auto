#include "transport_control.h"

using namespace std;

#define TRAJLENGTH 200
#define MAXDIS 99999
#define L 2.4

bool transport_Control::Init() {
  AINFO << "Transport_Control init";
  if(!GetProtoConfig(&control_setting_conf_)) {
    AERROR << "Unable to load conf file" << ConfigFilePath();
    return false;
  }
  ReadConfig();
  writer = node_->CreateWriter<ControlCommand>("/transport/control");

  gps_reader_=node_->CreateReader<Gps>("/transport/gps");
  traj_record_file.open("/apollo/modules/control/data/gps_record.csv", ios::out | ios::trunc);
  traj_record_file << "frame" << "," << "pgsnh" << ","
                   << "gpsnl" << "," << "gpseh" << "," << "gpsel" << "," << "heading_angle"
                   << "," << "yaw_rate" << "," << "gps_state" << "," << "gps_velocity" << ","
                   << "acceleration_forward" << "," << "acceleration_lateral" << "," << 
                   "acceleration_down" << "," << "pitch_angle" << "," << "velocity_down" <<
                   "," << "velocity_lateral" << "," << "velocity_forward" << "," << "roll_angle" << endl;
  frame = 0;
  // Init ControlCommand Writer
  ReadTraj();
  return true;
}

void transport_Control::ReadTraj() {
  //读取所有点的经纬度
  TrajFile.open("/apollo/modules/TrajFile.dat", ios::in);

  while (TrajFile.peek() != EOF) {
    double nh, nl, eh, el, vel;
    TrajFile >> nh >> nl >> eh >> el >> vel;
    trajinfo[0].push_back(nh);
    trajinfo[1].push_back(nl);
    trajinfo[2].push_back(eh);
    trajinfo[3].push_back(el);
    trajinfo[4].push_back(vel);
  }
  TrajIndex = 0;
}

void transport_Control::ReadConfig() {
  configinfo.look_ahead_dis = control_setting_conf_.lookaheaddis();
  configinfo.stanley_k = control_setting_conf_.stanleyk();
  configinfo.stanley_prop = control_setting_conf_.stanleyprop();
  configinfo.speed_mode = control_setting_conf_.speedmode();
  configinfo.desired_speed = control_setting_conf_.desiredspeed();
  configinfo.speed_k = control_setting_conf_.speedk();
}
void transport_Control::UpdateTraj(const std::shared_ptr<Gps>& msg0) {
  //将靠近的若干个点转移到车辆的坐标系中
  //寻找轨迹上一段范围内的距离本车最近的点
  double N_now = msg0->gpsnl() + msg0->gpsnh();
  double E_now = msg0->gpsel() + msg0->gpseh();
  double Azi_now = msg0->heading_angle() / 180 * M_PI;
  int lastindex = TrajIndex;
  double min_dis = MAXDIS;
  for (int i = lastindex;
       i < min(lastindex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[3][i] + trajinfo[4][i];
    double dis = SphereDis(E_now, N_now, E_point, N_point);
    if (dis < min_dis) {
      min_dis = dis;
      TrajIndex = i;
    }
  }
  //将该点附近的若干个点加入到自车坐标系中
  rel_loc[0].clear();
  rel_loc[1].clear();
  rel_loc[2].clear();
  for (int i = TrajIndex;
       i < min(TrajIndex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[2][i] + trajinfo[3][i];
    double dis = SphereDis(E_now, N_now, E_point, N_point);
    double azi = SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * cos(azi - Azi_now);
    double rel_y = dis * sin(azi - Azi_now);
    double vel = trajinfo[4][i];
    rel_loc[0].push_back(rel_x);
    rel_loc[1].push_back(rel_y);
    rel_loc[2].push_back(vel);
  }
}
int transport_Control::FindLookahead(double totaldis) {
  double dis = 0;
  int i = 1;
  for (i = 1; i < rel_loc[0].size(); i++) {
    double dx = (rel_loc[0][i] - rel_loc[0][i - 1]);
    double dy = (rel_loc[1][i] - rel_loc[1][i - 1]);
    dis += sqrt(dx * dx + dy * dy);
    if (dis > totaldis) {
      break;
    }
  }
  return i;
}

/*
  Reader Callback function
*/
bool transport_Control::Proc(const std::shared_ptr<Gps>& msg0) {
  if (frame == 65535) {
    frame = 0;
  }
  frame ++;
  traj_record_file << frame << "," << msg0->gpsnh() << "," << msg0->gpsnl() << "," << msg0->gpseh() 
                   << "," << msg0->gpsel() << "," << msg0->heading_angle() << "," << msg0->yaw_rate() 
                   << "," << msg0->gps_state() << "," << msg0->gps_velocity() 
                   << "," << msg0->acceleration_forward() << "," << msg0->acceleration_lateral() 
                   << "," << msg0->acceleration_down() << "," << msg0->pitch_angle() 
                   << "," << msg0->velocity_down() << "," << msg0->velocity_lateral() 
                   << "," << msg0->velocity_forward() << "," << msg0->roll_angle() << endl;

  double control_steer = 0;
  double control_acc = 0;
  UpdateTraj(msg0);
  // calculate steer
  control_steer = CaculateSteer(msg0);
  controlcmd.set_control_steer(control_steer);

  // calculate acc
  control_acc = CaculateAcc(msg0);
  controlcmd.set_control_acc(control_acc);
  AINFO << controlcmd.DebugString();

  writer->Write(controlcmd);
  return true;
}

/*
  Input Gps message
  Output Control_steer degree
*/
double transport_Control::CaculateSteer(
  const std::shared_ptr<Gps>& msg0) {
  double steer_wheel_angle = 0;
  //根据预瞄点计算横向转角
  int LookAheadIndex = FindLookahead(configinfo.look_ahead_dis);
  double long_distance = rel_loc[0][LookAheadIndex];
  double lat_distance = rel_loc[1][LookAheadIndex];
  double follow_angle =
      atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 / M_PI;
  //根据stanley计算转角
  double stanley_angle = 0;
  int validcheck = 0;
  stanley_angle =
      Stanley(configinfo.stanley_k, msg0->gps_velocity(), validcheck);
  // todo 增加前轮转角与方向盘转角的函数表达式。
  double StanleyProp = configinfo.stanley_prop;
  double front_wheel_angle =
      follow_angle * (1 - StanleyProp) + stanley_angle * StanleyProp;
  steer_wheel_angle = front_wheel_angle / 14.0 * 360.0;
  return steer_wheel_angle;
}

double transport_Control::Stanley(double k, double v, int& ValidCheck) {
  // use stanley method to calculate steer angle.
  ValidCheck = 1;
  // find point X=0
  int index = 0;
  for (int i = 0; i < rel_loc[0].size() - 1; i++) {
    if (abs(rel_loc[0][i]) < abs(rel_loc[0][index])) {
      index = i;
    }
  }
  // find angle at point X=0
  if (rel_loc[0][index + 1] - rel_loc[0][index] == 0) {
    ValidCheck = 0;
    return 0;
  }
  float phi_e = atan((rel_loc[1][index + 1] - rel_loc[1][index]) /
                     (rel_loc[0][index + 1] - rel_loc[0][index])) /
                M_PI * 180;
  // X[i] and X[i+1] may be same?

  // find y error at point X=0
  if (v <= 1) v = 1;
  float phi_y = atan(k * rel_loc[1][index] / v);
  return phi_e + phi_y;
}

//设置纵向期望速度
double transport_Control::CaculateAcc(
    const std::shared_ptr<Gps>& msg0) {
  double control_acc = 0;
  if (configinfo.speed_mode == 0) {
    // const speed mode;
    double N_now = msg0->gpsnl() + msg0->gpsnh();
    double E_now = msg0->gpsel() + msg0->gpseh();
    double N_start = trajinfo[0][0] + trajinfo[1][0];
    double E_start = trajinfo[2][0] + trajinfo[3][0];
    int length = trajinfo[0].size();
    double N_end = trajinfo[0][length] + trajinfo[1][length];
    double E_end = trajinfo[2][length] + trajinfo[3][length];
    double DisToStart = SphereDis(E_now, N_now, E_start, N_start);
    double DisToEnd = SphereDis(E_now, N_now, E_end, N_end);
    control_acc = min(DisToStart, DisToEnd) * configinfo.speed_k;
    control_acc = min(configinfo.desired_speed, control_acc);

  } else if (configinfo.speed_mode == 0) {
    // Traj speed mode
    control_acc = trajinfo[4][TrajIndex];
  }

  return control_acc;
}

void transport_Control::Clear() {
  TrajFile.close();
  traj_record_file.close();
}
