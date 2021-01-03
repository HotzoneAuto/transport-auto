#include "transport_control.h"

#define TRAJLENGTH 400
#define MAXDIS 99999
#define L 2.4

bool transport_Control::Init() {
  AINFO << "Transport_Control init";
  if (!GetProtoConfig(&control_setting_conf_)) {
    AERROR << "Unable to load control_setting_conf file" << ConfigFilePath();
    return false;
  }
  ReadCanConfig();
  AINFO << "After read can config, SpeedThreshold = " << SpeedThreshold;
  writer = node_->CreateWriter<ControlCommand>("/transport/control");
  if(control_setting_conf_.trajmode() == 0){
    traj_record_file.open("/apollo/modules/control/data/gps_record.csv", std::ios::out | std::ios::trunc);
    traj_record_file << "frame" << "," << "gpsnh" << ","
                  << "gpsnl" << "," << "gpseh" << "," << "gpsel" << "," << "heading_angle"
                  << "," << "yaw_rate" << "," << "gps_state" << "," << "gps_velocity" << ","
                  << "acceleration_forward" << "," << "acceleration_lateral" << "," << 
                  "acceleration_down" << "," << "pitch_angle" << "," << "velocity_down" <<
                  "," << "velocity_lateral" << "," << "velocity_forward" << "," << "roll_angle" << std::endl;
  }else if(control_setting_conf_.trajmode() == 1){
    ReadTraj();
  }

  // Init ControlCommand Writer
  return true;
}

void transport_Control::ReadTraj() {
  //读取所有点的经纬度
  traj_record_file.open("/apollo/modules/control/data/gps_record.csv",
                        std::ios::in);
  char linestr[500] = {0};
  traj_record_file.getline(linestr, 500);
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
    trajinfo[0].push_back(gpsnh);
    trajinfo[1].push_back(gpsnl);
    trajinfo[2].push_back(gpseh);
    trajinfo[3].push_back(gpsel);
    trajinfo[4].push_back(velocity);
  }
  traj_record_file.close();
}

//将靠近的若干个点转移到车辆的坐标系中
//寻找轨迹上一段范围内的距离本车最近的点
void transport_Control::UpdateTraj(const std::shared_ptr<Gps>& msg0) {
  double N_now = msg0->gpsnl() + msg0->gpsnh();
  double E_now = msg0->gpsel() + msg0->gpseh();
  double Azi_now = msg0->heading_angle() / 180 * M_PI;
  int lastindex = TrajIndex;
  double min_dis = MAXDIS;
  for (int i = lastindex;
       i < std::min(lastindex + TRAJLENGTH/2, (int)trajinfo[0].size()); i++) {
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
  AINFO<<"TrajIndex= "<<TrajIndex << "  MINDIS=" <<min_dis;
  //将该点附近的若干个点加入到自车坐标系中
  rel_loc[0].clear();
  rel_loc[1].clear();
  rel_loc[2].clear();
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
    dis += std::sqrt(dx * dx + dy * dy);
    if (dis > totaldis) {
      break;
    }
  }
  if(i == (int)rel_loc[0].size()) i = (int)rel_loc[0].size()-1;
  return i;
}

// Reader Callback function
bool transport_Control::Proc(const std::shared_ptr<Gps>& msg0) {
  if (frame == 65535) {
    frame = 0;
  }
  //frame++;
  if (control_setting_conf_.trajmode() == 0) {
    static double last_gpsn=0,last_gpse=0;
    double gpsn=msg0->gpsnh()+msg0->gpsnl();
    double gpse=msg0->gpseh()+msg0->gpsel();
    double dis_to_last_point=
        apollo::drivers::gps::SphereDis(last_gpse,last_gpsn,gpse,gpsn);
    if(last_gpsn==0 || (last_gpsn!=0 && dis_to_last_point > 0.1 )){
      frame++;
      traj_record_file << frame << "," << msg0->gpsnh() << "," << msg0->gpsnl()
                  << "," << msg0->gpseh() << "," << msg0->gpsel() << ","
                  << msg0->heading_angle() << "," << msg0->yaw_rate() << ","
                  << msg0->gps_state() << "," << msg0->gps_velocity() << ","
                  << msg0->acceleration_forward() << ","
                  << msg0->acceleration_lateral() << ","
                  << msg0->acceleration_down() << "," << msg0->pitch_angle()
                  << "," << msg0->velocity_down() << ","
                  << msg0->velocity_lateral() << ","
                  << msg0->velocity_forward() << "," << msg0->roll_angle()
                  << std::endl;
      last_gpsn = gpsn;
      last_gpse = gpse;
    }

  } else if (control_setting_conf_.trajmode() == 1) {
    double control_steer = 0;
    double control_acc = 0;

    UpdateTraj(msg0);
    if (rel_loc[0].size() > 10) {
      // near destination
      // calculate steer
      control_steer = CaculateSteer(msg0);
      if (control_steer > 850.0) {
        control_steer = 850.0;
      } else if (control_steer < -850.0) {
        control_steer = -850.0;
      }
      // controlcmd.set_control_steer(control_steer);
      controlcmd.set_control_steer(-control_steer);

      // calculate acc
      control_acc = CaculateAcc(msg0);
      controlcmd.set_control_acc(control_acc);
      AINFO << controlcmd.DebugString();
    } else {
      controlcmd.set_control_steer(0);
      controlcmd.set_control_acc(0);
    }
    writer->Write(controlcmd);
  }

  return true;
}

/*
  Input Gps message
  Output Control_steer degree
*/
double transport_Control::CaculateSteer(const std::shared_ptr<Gps>& msg0) {
  double steer_wheel_angle = 0;
  //根据预瞄点计算横向转角
  int LookAheadIndex = FindLookahead(control_setting_conf_.lookaheaddis());
  double long_distance = rel_loc[0][LookAheadIndex];
  double lat_distance = rel_loc[1][LookAheadIndex];
  double follow_angle =
      std::atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 /
      M_PI;
  AINFO<<"LookAheadIndex: "<< LookAheadIndex <<" lat distance: "<< lat_distance << " long distance: "<< long_distance;
  //根据stanley计算转角
  double stanley_angle = 0;
  int validcheck = 0;
  // stanley_angle =
  //     Stanley(configinfo.stanley_k, msg0->gps_velocity(), validcheck);
  // todo 增加前轮转角与方向盘转角的函数表达式。
  double StanleyProp = control_setting_conf_.stanleyprop();
  // double front_wheel_angle =
  //     follow_angle * (1 - StanleyProp) + stanley_angle * StanleyProp;
  double front_wheel_angle = follow_angle;
  steer_wheel_angle = front_wheel_angle / 14.0 * 360.0;
  return steer_wheel_angle;
}

double transport_Control::Stanley(double k, double v, int& ValidCheck) {
  // use stanley method to calculate steer angle.
  ValidCheck = 1;
  // find point X=0
  int index = 0;
  for (int i = 0; i < rel_loc[0].size() - 1; i++) {
    if (std::abs(rel_loc[0][i]) < std::abs(rel_loc[0][index])) {
      index = i;
    }
  }
  // find angle at point X=0
  if (rel_loc[0][index + 1] - rel_loc[0][index] == 0) {
    ValidCheck = 0;
    return 0;
  }
  float phi_e = std::atan((rel_loc[1][index + 1] - rel_loc[1][index]) /
                          (rel_loc[0][index + 1] - rel_loc[0][index])) /
                M_PI * 180;
  // X[i] and X[i+1] may be same?

  // find y error at point X=0
  if (v <= 1) v = 1;
  float phi_y = std::atan(k * rel_loc[1][index] / v);
  return phi_e + phi_y;
}

//设置纵向期望速度
double transport_Control::CaculateAcc(const std::shared_ptr<Gps>& msg0) {
  double control_acc = 0;
  if (control_setting_conf_.speedmode() == 0) {
    // const speed mode;
    double N_now = msg0->gpsnl() + msg0->gpsnh();
    double E_now = msg0->gpsel() + msg0->gpseh();
    double N_start = trajinfo[0][0] + trajinfo[1][0];
    double E_start = trajinfo[2][0] + trajinfo[3][0];
    int length = trajinfo[0].size();
    double N_end = trajinfo[0][length] + trajinfo[1][length];
    double E_end = trajinfo[2][length] + trajinfo[3][length];
    double DisToStart =
        apollo::drivers::gps::SphereDis(E_now, N_now, E_start, N_start);
    double DisToEnd =
        apollo::drivers::gps::SphereDis(E_now, N_now, E_end, N_end);
    if (DisToStart < DisToEnd) {
      control_acc = std::max(DisToStart * control_setting_conf_.speedk(), SpeedThreshold);
      AINFO << "When DisToStart < DisToEnd, control_acc = " << control_acc;
    } else {
      control_acc = DisToEnd * control_setting_conf_.speedk();
    }
    control_acc = std::min(control_setting_conf_.desiredspeed(), control_acc);

  } else if (control_setting_conf_.speedmode() == 1) {
    // Traj speed mode
    control_acc = trajinfo[4][TrajIndex];
  }

  return control_acc;
}

void transport_Control::Clear() { traj_record_file.close(); }

void transport_Control::ReadCanConfig() {
  std::ifstream f;
  f.open("/apollo/modules/canbus/conf/transport_can_conf.config");
  if (f.is_open()) {
    AINFO << "Can Config File Opened";
    while (!f.eof()) {
      std::string SettingName;
      f >> SettingName;
      if (SettingName == "SpeedThreshold") {
        f >> SpeedThreshold;
        AINFO << "In ReadCanConfig(), SpeedThreshold= " << SpeedThreshold;
      }
    }
    f.close();
  } else
    AERROR << "transport_can_conf.config Missing";
}
