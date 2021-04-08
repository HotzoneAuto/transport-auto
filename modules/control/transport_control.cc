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
  // Init ControlCommand Writer
  writer = node_->CreateWriter<ControlCommand>("/transport/control");

  return true;
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
  if (i == (int)rel_loc[0].size()) i = (int)rel_loc[0].size() - 1;
  return i;
}

// Reader Callback function
bool transport_Control::Proc(const std::shared_ptr<Trajectory>& msg0) {
  rel_loc[0].clear();
  rel_loc[1].clear();
  rel_loc[2].clear();
  rel_loc[3].clear();
  for (int i = 0; i < msg0->points_size(); i++) {
    rel_loc[0].push_back(msg0->points(i).rel_x());
    rel_loc[1].push_back(msg0->points(i).rel_y());
    rel_loc[2].push_back(msg0->points(i).rel_vel());
    rel_loc[3].push_back(msg0->points(i).timestamp());
  }

  double control_steer = 0;
  double control_acc = 0;

  if (rel_loc[0].size() > 10) {
    // near destination
    // calculate steer
    control_steer = CaculateSteer(msg0);
    if (control_steer > 720.0) {
      control_steer = 720.0;
    } else if (control_steer < -720.0) {
      control_steer = -720.0;
    }
    controlcmd.set_control_steer(-control_steer);

    // calculate acc
    control_acc = CaculateAcc(msg0);
    controlcmd.set_control_acc(control_acc);
    AINFO << controlcmd.DebugString();
  } else {
    controlcmd.set_control_steer(0);
    controlcmd.set_control_acc(0);
  }
  if (msg0->gps_state() != 4) {
    controlcmd.set_control_steer(0);
    controlcmd.set_control_acc(0);
  }
  writer->Write(controlcmd);
  return true;
}

/*
  Input Gps message
  Output Control_steer degree
*/
double transport_Control::CaculateSteer(
    const std::shared_ptr<Trajectory>& msg0) {
  double steer_wheel_angle = 0;
  //根据预瞄点计算横向转角
  int LookAheadIndex = FindLookahead(control_setting_conf_.lookaheaddis());
  double long_distance = rel_loc[0][LookAheadIndex]+ control_setting_conf_.posoffset();
  // offset = +1 表示后轴在GPS后方1m
  double lat_distance = rel_loc[1][LookAheadIndex];
  double distance= sqrt(long_distance*long_distance+lat_distance*lat_distance);
  double follow_angle =
      std::atan(2 * L * lat_distance / (distance * distance)) * 180 /
      M_PI;
  AINFO << "LookAheadIndex: " << LookAheadIndex
        << " lat distance: " << lat_distance
        << " long distance: " << long_distance;
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
double transport_Control::CaculateAcc(const std::shared_ptr<Trajectory>& msg0) {
  double control_acc = 0;
  if (control_setting_conf_.speedmode() == 0) {
    // const speed mode;
    double DisToStart = msg0->dis_to_start();
    double DisToEnd = msg0->dis_to_end();
    if (DisToStart < DisToEnd) {
      control_acc = std::max(DisToStart * control_setting_conf_.speedk(),
                             control_setting_conf_.speedthreshold());
      AINFO << "When DisToStart < DisToEnd, control_acc = " << control_acc;
    } else {
      control_acc = DisToEnd * control_setting_conf_.speedk();
    }
    control_acc = std::min(control_setting_conf_.desiredspeed(), control_acc);
  } else if (control_setting_conf_.speedmode() == 1) {
    // Traj speed mode
    control_acc = msg0->control_acc();
  }

  return control_acc;
}
