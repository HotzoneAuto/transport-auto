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

  gps_reader_ = node_->CreateReader<Gps>(
      "/transport/gps",
      [this](const std::shared_ptr<Gps>& msg) { 
        gps_.CopyFrom(*msg); 
        vol_cur = gps_.gps_velocity() * 3.6;
        v_lateral = gps_.velocity_lateral();
        v_forward = gps_.velocity_forward();
        heading_angle_now = gps_.heading_angle();
        yaw_rate_now = gps_.yaw_rate();
        AINFO << "After read gps, vol_cur = " << vol_cur;
        AINFO << "v_lateral = " << v_lateral;
        AINFO << "v_forward = " << v_forward;
      });

  AINFO << "transport_Control Init OK!";

  nanotime_last = Time::Now().ToNanosecond();
  nanotime_init = nanotime_last;

  //Experiment Record File
    fname="/apollo/modules/control/data/exp_record.csv";
    file_csv.open_file(fname);
    std::string msg_w =
        "frame,e_y,e_y_dot,e_phi,e_phi_dot,index,u1/deg,u1_0,u1_1,u1_2,u1_3,u2,curvature,v_x,v_y,timestamp";
    file_csv.write_file(msg_w);

  return true;
}
void transport_Control::Clear(){
    file_csv.close_file();
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
  rel_loc[4].clear();
  rel_loc[5].clear();
  rel_loc[6].clear();
  for (int i = 0; i < msg0->points_size(); i++) {
    rel_loc[0].push_back(msg0->points(i).rel_x());
    rel_loc[1].push_back(msg0->points(i).rel_y());
    rel_loc[2].push_back(msg0->points(i).rel_vel());
    rel_loc[3].push_back(msg0->points(i).yaw_rate());
    rel_loc[4].push_back(msg0->points(i).heading_angle());
    rel_loc[5].push_back(msg0->points(i).curvature());
    rel_loc[6].push_back(msg0->points(i).timestamp());
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

    control_accpedal_flag=1;
    control_brkpedal_flag=0;
    control_clupedal_flag=0;
    control_accpedal=50;
    control_brkpedal=0;
    control_clupedal=0;

    AINFO << controlcmd.DebugString();
  } else {
    control_acc = 0;
    controlcmd.set_control_steer(0);
    controlcmd.set_control_acc(control_acc);

  }
  if (msg0->gps_state() != 4) {
    control_acc = 0;
    controlcmd.set_control_steer(0);
    controlcmd.set_control_acc(control_acc);
  }

  nanotime_now = Time::Now().ToNanosecond();
  AINFO << "nanotime_last = " << nanotime_last;
  AINFO << "nanotime_now = " << nanotime_now;

  if (nanotime_last == nanotime_init) {
    delta_t = 0;
  } else {
    delta_t = (nanotime_now - nanotime_last) * 1e-9;
  }

  //CalculatePedalGear(control_acc, delta_t);

  controlcmd.set_control_accpedal_flag(control_accpedal_flag);
  controlcmd.set_control_brkpedal_flag(control_brkpedal_flag);
  controlcmd.set_control_clupedal_flag(control_clupedal_flag); 
  controlcmd.set_control_accpedal(control_accpedal);
  controlcmd.set_control_brkpedal(control_brkpedal);
  controlcmd.set_control_clupedal(control_clupedal);
  controlcmd.set_control_gear(control_gear);

  writer->Write(controlcmd);

  nanotime_last = Time::Now().ToNanosecond();

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
 /* double follow_angle =
      std::atan(2 * L * lat_distance / (distance * distance)) * 180 /
      M_PI;*/
  double follow_angle=LookAheadPredict() *180 /M_PI;

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

double transport_Control::LookAheadPredict() {
  // find longitudinal nearest point
  int index = 0;
  for (int i = 0; i < rel_loc[0].size() - 1; i++) {
    if (std::abs(rel_loc[0][i]) < std::abs(rel_loc[0][index])) {
      index = i;
    }
  }

  double e_y = rel_loc[1][index];

  // TODO: New e_phi version by xingyu
  // double e_phi = std::atan((rel_loc[1][index + 1] - rel_loc[1][index]) / (rel_loc[0][index + 1] - rel_loc[0][index]) / M_PI * 180);
  
  // find e_phi.  use 5 point.
/*
  double e_phi=0;
  int e_phi_num=0;
  while(index+e_phi_num +1 < rel_loc[0].size() && e_phi_num+1<=5){
    e_phi_num++;
    int tmpindex=e_phi_num+index;    
    double dx = rel_loc[0][tmpindex]-rel_loc[0][index];
    double dy = rel_loc[1][tmpindex]-rel_loc[1][index];
    double d_e_phi = acos(dx/sqrt(dx*dx+dy*dy));
    e_phi += d_e_phi;
  }
  e_phi /= e_phi_num;
*/
  // find e_phi by history

  double phi_des = rel_loc[4][index];
  double e_phi = -(heading_angle_now - phi_des) / 180 * M_PI;

  double v_x = v_forward;
  double v_y = v_lateral;
  double e_y_dot = v_y + v_x * e_phi;

  // TODO: check all angle and velocity units
  double yaw_rate_des = rel_loc[3][index];
  double e_phi_dot = (yaw_rate_now - yaw_rate_des) / 180 * M_PI;

  double u1_0 = kb[0] * e_y;
  double u1_1 = kb[1] * e_y_dot;
  double u1_2 = kb[2] * e_phi;
  double u1_3 = kb[3] * e_phi_dot;

  double u1 = u1_0 + u1_1 + u1_2 + u1_3;
  double u2 = 0;
/*
  for (int i = 0; i < sizeof(kf); i++) {
    u2 += kf[i] * rel_loc[5][index + i];
  }
*/
  //record e_y e_phi e_y_dot e_phi_dot
    if (frame == 65535) {
      frame = 0;
    }
    frame++;

    std::string msg_w = std::to_string(frame) + "," + std::to_string(e_y) 
                    + "," + std::to_string(e_y_dot) 
                    + "," + std::to_string(e_phi) 
                    + "," + std::to_string(e_phi_dot)
                    + "," + std::to_string(index)
                    + "," + std::to_string(u1 / M_PI * 180)
                    + "," + std::to_string(u1_0)
                    + "," + std::to_string(u1_1)
                    + "," + std::to_string(u1_2)
                    + "," + std::to_string(u1_3)
                    + "," + std::to_string(u2/ M_PI * 180)
                    + "," + std::to_string(rel_loc[5][index])
                    + "," + std::to_string(v_x)
                    + "," + std::to_string(v_y)
                    + "," + std::to_string(Time::Now().ToNanosecond());
    file_csv.write_file(msg_w);


  //double front_wheel_angle = u1 + u2;
  double front_wheel_angle = u1;
  return front_wheel_angle;
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

//设置三个踏板开度和期望换挡动作
void transport_Control::CalculatePedalGear(double vol_exp, double delta_t) {
  float ths_dif = control_setting_conf_.speederrorthreshold();
  float ths_exp = control_setting_conf_.speedthreshold();
  AINFO << "ths_exp = " << ths_exp << ", ths_dif = " << ths_dif
        << ", control_setting_conf_.idlespeed() = "
        << control_setting_conf_.idlespeed();
  AINFO << "control_setting_conf_.clutchset() = "
        << control_setting_conf_.clutchset();

  if ((wait_flag == 1) || (finishstop_flag == 1) ||
      ((vol_exp == 0) && (start_flag == 1))) {
    control_flag = 1;
    AINFO << "control_flag is set as: 1";
  } else if ((start_flag == 1) && (vol_exp > ths_exp - 0.1)) {
    control_flag = 2;
    AINFO << "control_flag is set as: 2";
  } else if (vol_exp < control_setting_conf_.idlespeed()) {
    control_flag = 5;
    AINFO << "control_flag is set as: 5";
  } else if ((vol_cur < ths_exp) || (vol_exp - vol_cur) > ths_dif) {
    control_flag = 3;
    AINFO << "control_flag is set as: 3";
  } else if ((vol_exp - vol_cur) < ths_dif && (vol_cur > ths_exp)) {
    control_flag = 4;
    AINFO << "control_flag is set as: 4";
  }

  AINFO << "Before switch cases, control_flag = " << control_flag;
  if (control_flag) {
    switch (control_flag) {
      case 1:
        AINFO << "Into case 1";
        if (wait_flag == 1) {
          wait_time += delta_t;
          AINFO << "delta_t = " << delta_t;
          AINFO << "wait_time = " << wait_time;
          if ((wait_time > control_setting_conf_.waitingtime()) &&
              (finishstop_flag == 0)) {
            wait_flag = 0;
            wait_time = 0;
            start_flag = 1;
            AINFO << "start_flag is changed to 1.";
          }
          /* wait_count++;
          if ((wait_count > control_setting_conf_.waitingtime() / 0.02) &&
              (finishstop_flag == 0)) {
            wait_flag = 0;
            wait_count = 0;
            start_flag = 1;
          }*/
        }

        control_accpedal_flag = 0;
        control_brkpedal_flag = 1;
        control_clupedal_flag = 1;

        control_clupedal = control_setting_conf_.clutchset();
        control_brkpedal = control_setting_conf_.brakeset();
        control_accpedal = 0;

        // staying in N
        control_gear = 0;

        cluopen_last = control_setting_conf_.clutchset();
        brkopen_last = control_setting_conf_.brakeset();

        break;
      // start mode
      case 2:
        // brake set
        AINFO << "Into start mode, control_flag = " << control_flag;

        control_accpedal_flag = 0;
        control_brkpedal_flag = 1;
        control_clupedal_flag = 1;

        control_brkpedal = 0;
        control_accpedal = 0;
        AINFO << "brkpedalopenreq and accpedalopenreq are set as: 0";
        brkopen_last = 0;

        delta_clu = control_setting_conf_.clutchreleaserate() * delta_t;
        if (cluopen_last > control_setting_conf_.clutchthreshold()) {
          control_clupedal = int(cluopen_last - delta_clu);          
          AINFO << "clupedalopenreq is set as: " << control_clupedal;
        } else {
          control_clupedal = 0;
          AINFO << "clupedalopenreq is set as: 0";
          if (vol_cur > control_setting_conf_.idlespeed() / 2) {
            start_flag = 0;
          }
        }
        cluopen_last = control_clupedal;

        // N to 1
        control_gear = 1;   

        break;
      // normal mode
      case 3:
        // P control
        AINFO << "Into normal mode, control_flag = " << control_flag;

        control_accpedal_flag = 1;
        control_brkpedal_flag = 0;
        control_clupedal_flag = 0;

        control_accpedal = int(vol_exp / control_setting_conf_.kspeedthrottle() +
                (vol_exp - vol_cur) * control_setting_conf_.kdrive());
        AINFO << "accpedalopenreq is set as: " << control_accpedal;
        control_brkpedal = 0;
        control_clupedal = 0;
        AINFO << "brkpedalopenreq and clupedalopenreq are set as: 0";
        brkopen_last = 0;
        cluopen_last = 0;

        // staying in 1
        control_gear = 1;

        break;
      // emergency mode
      case 4:
        // P control
        AINFO << "Into emergency mode, control_flag = " << control_flag;

        control_accpedal_flag = 0;
        control_brkpedal_flag = 1;
        control_clupedal_flag = 0;

        control_brkpedal = int(-(vol_exp - vol_cur) * control_setting_conf_.kbrake());
        AINFO << "brkpedalopenreq is set as: " << control_brkpedal;
        control_accpedal = 0;
        control_clupedal = 0;
        AINFO << "accpedalopenreq and clupedalopenreq are set as: 0";

        brkopen_last = control_brkpedal;
        cluopen_last = control_clupedal;

        // staying in 1
        control_gear = 1;

        break;
      // stop mode
      case 5:
        // li he
        AINFO << "Into stop mode, control_flag = " << control_flag;

        control_accpedal_flag = 0;
        control_brkpedal_flag = 1;
        control_clupedal_flag = 1;

        control_clupedal = control_setting_conf_.clutchset();
        AINFO << "clupedalopenreq is set as: " << control_clupedal;
        cluopen_last = control_clupedal;

        control_accpedal = 0;
        AINFO << "accpedalopenreq is set as: " << 0;

        delta_brk = control_setting_conf_.brakeapplyrate() * delta_t;
        if (brkopen_last < control_setting_conf_.brakeset()) {
          control_brkpedal = int(brkopen_last + delta_brk);
          AINFO << "brkpedalopenreq is set as: " << control_brkpedal;          
        } else {
          control_brkpedal = control_setting_conf_.brakeset();
          AINFO << "brkpedalopenreq is set as: " << control_brkpedal;
          finishstop_flag = 1;
        }
        brkopen_last = control_brkpedal;

        // staying in 1
        control_gear = 1;

        break;
      default:
        AINFO << "In default, control_flag = " << control_flag;
        break;
    }
  }
}

