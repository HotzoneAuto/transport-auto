#include "transport_control.h"

#define TRAJLENGTH 400
#define MAXDIS 99999

bool transport_Control::Init() {
  AINFO << "Transport_Control init";
  if (!GetProtoConfig(&control_setting_conf_)) {
    AERROR << "Unable to load control_setting_conf file" << ConfigFilePath();
    return false;
  }
  // Init ControlCommand Writer
  writer = node_->CreateWriter<ControlCommand>("/transport/control");
  // Init ControlFlag Writer and Write initial state
  flag_writer = node_->CreateWriter<ControlFlag>("/transport/controlflag");
  controlflag.Clear();
  AINFO<<controlflag.DebugString();

  //Init TrajNumber
  std::string planning_conf_path="/apollo/modules/planning/conf/planning_replay_conf.pb.txt";
  if( !GetProtoFromFile(planning_conf_path,&planning_setting_conf_) ){
    AERROR << "Unable to load conf file" << planning_conf_path;
    return false;
  }
  controlflag.set_traj_number(planning_setting_conf_.trajnumber());


  flag_writer->Write(controlflag);

  gps_reader_ = node_->CreateReader<Gps>(
      "/transport/gps",
      [this](const std::shared_ptr<Gps>& msg) { 
        gps_.CopyFrom(*msg); 
        vol_cur = gps_.gps_velocity() * 3.6;
        AINFO << "After read gps, vol_cur = " << vol_cur;
      });

  AINFO << "transport_Control Init OK!";

  nanotime_last = Time::Now().ToNanosecond();
  nanotime_init = nanotime_last;

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
bool transport_Control::Proc(const std::shared_ptr<Trajectory>& msg0,
        const std::shared_ptr<ChassisDetail>& msg1) {
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
  AINFO << "Start Control Logic";
  CalculatePedalGear(control_acc, delta_t,*msg0,*msg1);
  AINFO << "Finish Control Logic";
  controlcmd.set_control_accpedal_flag(control_accpedal_flag);
  controlcmd.set_control_brkpedal_flag(control_brkpedal_flag);
  controlcmd.set_control_clupedal_flag(control_clupedal_flag); 
  controlcmd.set_control_accpedal((int)control_accpedal);
  controlcmd.set_control_brkpedal((int)control_brkpedal);
  controlcmd.set_control_clupedal((int)control_clupedal);
  controlcmd.set_control_gear(control_gear);
  AINFO<<controlcmd.DebugString();  
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
  int LookAheadIndex = FindLookahead(control_setting_conf_.lookaheaddis() - control_setting_conf_.posoffset());
  double long_distance = rel_loc[0][LookAheadIndex]+ control_setting_conf_.posoffset();
  // offset = +1 表示后轴在GPS后方1m
  double lat_distance = rel_loc[1][LookAheadIndex];
  double distance= sqrt(long_distance*long_distance+lat_distance*lat_distance);
  double follow_angle =
      std::atan(2 * control_setting_conf_.wheelbase() * lat_distance / (distance * distance)) * 180 /
      M_PI;
  AINFO << "LookAheadIndex: " << LookAheadIndex
        << " lat distance: " << lat_distance
        << " long distance: " << long_distance;
  //根据stanley计算转角
  double stanley_angle = 0;
  int validcheck = 0;
   stanley_angle =
       Stanley(control_setting_conf_.stanleyk(), vol_cur, validcheck) * 180 / M_PI;
  double StanleyProp = control_setting_conf_.stanleyprop();
  double front_wheel_angle =
      follow_angle * (1 - StanleyProp) + stanley_angle * StanleyProp;
  //double front_wheel_angle = follow_angle;
  steer_wheel_angle = front_wheel_angle / 14.0 * 360.0;
  return steer_wheel_angle;
}

double transport_Control::Stanley(double k, double v, int& ValidCheck) {
  // use stanley method to calculate steer angle.
  ValidCheck = 1;
  // find point X=0
  int index = 0;
  double front_wheel_offset=control_setting_conf_.wheelbase()-control_setting_conf_.posoffset();
  //寻找距离车辆前轴最近点
  double mindis=99999;
  for (int i = 0; i < rel_loc[0].size() - 1; i++) {
    double dis=sqrt( (rel_loc[0][i]-front_wheel_offset)*(rel_loc[0][i]-front_wheel_offset) 
              + rel_loc[1][i]*rel_loc[1][i]);
    if (dis < mindis) {
      mindis = dis;
      index = i;
    }
  }
  // find angle at point i  
  // use point i+1 ~ i+5
  double phi_e=0;
  for( int i = 1 ; index + i < rel_loc[0].size() && i<=5; i++ ){
    double dx = rel_loc[0][index+i] - rel_loc[0][index];
    double dy = rel_loc[0][index+i] - rel_loc[0][index];
    double phi_temp = std::acos( dx / sqrt(dx*dx+dy*dy) );
    phi_e += phi_temp;
  }
  phi_e /= 5;

  // find y error at point X=0
  if (v <= 0.5) v = 0.5;

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

//设置三个踏板开度和期望换挡动作
void transport_Control::CalculatePedalGear(double vol_exp, double delta_t,Trajectory msg0,ChassisDetail msg1) {
  float vdif_ths = control_setting_conf_.speederrorthreshold();
  float vol_ths = control_setting_conf_.speedthreshold();
  float vol_idle = control_setting_conf_.idlespeed();
  AINFO << "vol_ths = " << vol_ths << ", vdif_ths = " << vdif_ths
        << ", vol_idle = "
        << vol_idle;
  AINFO << "control_setting_conf_.clutchsethigh() = "
        << control_setting_conf_.clutchsethigh();
  //记录master标志位
  controlflag.set_stopfrmaster_flag( msg1.stopfrmaster_flag() );
  controlflag.set_missionfrmaster_flag( msg1.missionfrmaster_flag() );
  controlflag.set_trajfrmaster_flag( msg1.trajfrmaster_flag() );
  //处理标志位的赋值
  //轨迹编号
  if(controlflag.traj_number()==1){
    if( (msg1.trajfrmaster_flag()==2 || msg1.loadfrdigger_flag()==2) && controlflag.control_flag()==2)
    controlflag.set_traj_number(2);
  } else if(controlflag.traj_number()==2){
    if( (msg1.trajfrmaster_flag()==1 || msg1.loadfrdigger_flag()==1) && controlflag.control_flag()==2)
    controlflag.set_traj_number(1);
  }
  //任务标志赋值
  if(controlflag.mission_flag()==0){
    if(msg1.missionfrmaster_flag()==1 || 
        (msg1.missionfrmaster_flag()==2 && (msg1.reqfrdigger_flag()==1 || msg1.loadfrdigger_flag()==2)))
      controlflag.set_mission_flag(1);
  } else if(controlflag.mission_flag()==1){
    if(msg0.dis_to_end()<=control_setting_conf_.parkthreshold())
      controlflag.set_mission_flag(0);
  }
  //停车标志赋值
  if(controlflag.stopfrmaster_flag()==3){
    controlflag.set_stop_flag(3);
  }else if(controlflag.stopfrmaster_flag()==1){
    controlflag.set_stop_flag(1);
  }else if(controlflag.stopfrmaster_flag()==2 || controlflag.stopfrdigger_flag()==1 || vol_exp < vol_idle){
    controlflag.set_stop_flag(2);
  }else{
    controlflag.set_stop_flag(0);
  }
  //启动标记赋值
  if(controlflag.start_flag()==1 && ( controlflag.stop_flag()!=0)){
    controlflag.set_start_flag(0);
  }else if(controlflag.start_flag()==0 && controlflag.park_flag()==1 && controlflag.stop_flag()==0 && 
            controlflag.wait_flag()==0 &&controlflag.mission_flag()==1 && vol_exp>=vol_idle){
    controlflag.set_start_flag(1);
  }
  //反馈标志赋值
  if(controlflag.control_flag()==1){
    controlflag.set_statetodigger_flag(0);
  }else if(controlflag.control_flag()>=3 && controlflag.control_flag()<=6 && controlflag.traj_number()==2){
    controlflag.set_statetodigger_flag(1);
  }else if(controlflag.control_flag()==2 && controlflag.traj_number()==2){
    controlflag.set_statetodigger_flag(2);
  }else if(controlflag.control_flag()>=3 && controlflag.control_flag()<=6 && controlflag.traj_number()==1){
    controlflag.set_statetodigger_flag(3);
  }else if(controlflag.control_flag()==2 && controlflag.traj_number()==1){
    controlflag.set_statetodigger_flag(4);
  }else if(controlflag.control_flag()==0){
    controlflag.set_statetodigger_flag(15);
  }

  //行驶模式切换
  if(controlflag.stop_flag()==3){
    controlflag.set_control_flag(0);//不控制模式
  }else if(controlflag.park_flag()==1 && controlflag.stop_flag()==1){
    controlflag.set_control_flag(1);//驻车模式
  }else if(controlflag.park_flag()==1 && controlflag.stop_flag()==2){
    controlflag.set_control_flag(2);//待机模式
  }else if(controlflag.wait_flag()==0 && controlflag.start_flag()==1){
    controlflag.set_control_flag(3);//起动模式
  }else if(controlflag.park_flag()==0 && (controlflag.stop_flag()==1 ||controlflag.stop_flag()==2)){
    controlflag.set_control_flag(4);//停车模式
  }else if(controlflag.park_flag()==0 && controlflag.stop_flag()==0 && (vol_cur<vol_ths || vol_exp-vol_cur>vdif_ths)){
    controlflag.set_control_flag(5);//行驶模式
  }else if(controlflag.park_flag()==0 && controlflag.stop_flag()==0 && (vol_cur>=vol_ths && vol_exp-vol_cur<vdif_ths)){
    controlflag.set_control_flag(6);//制动模式
  }

  switch(controlflag.control_flag()){
    case 0://不控制模式
      controlcmd.set_control_steer_flag(0); //转向不控制
      controlflag.set_start_flag(0);
      control_accpedal_flag=0;
      control_brkpedal_flag=0;
      control_clupedal_flag=0;
      control_accpedal=0;
      control_brkpedal=0;
      control_clupedal=0;
      control_gear=0;
      break;
    case 1://驻车模式
      controlcmd.set_control_steer_flag(0); //转向不控制
      control_accpedal_flag=0;
      control_brkpedal_flag=1;
      control_clupedal_flag=1;
      control_brkpedal = control_setting_conf_.brakeset();
      control_accpedal = 0;
      control_clupedal = control_setting_conf_.clutchsethigh();
      if(control_gear==0){
        control_clupedal=0;
      }else if(control_gear==1){
        control_gear=2;
        controlflag.set_wait_flag(1);
        wait_time += delta_t;
        if(wait_time > control_setting_conf_.waitingtimelong()){
          control_gear=0;
          control_clupedal = 0;
          controlflag.set_wait_flag(0);
          wait_time=0;
        }
      }
      break;
    case 2://待机模式
      controlcmd.set_control_steer_flag(1); //转向控制
      control_accpedal_flag=0;
      control_brkpedal_flag=1;
      control_clupedal_flag=1;
      control_brkpedal = control_setting_conf_.brakeset();
      control_accpedal = 0;
      control_clupedal = control_setting_conf_.clutchsethigh();
      if(control_gear!=1){
        controlflag.set_wait_flag(1);
        wait_time += delta_t;
        if(wait_time > control_setting_conf_.waitingtimeshort()){
          control_gear=1;
          wait_time=0;
        }
      }else if(control_gear==1 && controlflag.wait_flag()==1){
        wait_time += delta_t;
        if(wait_time > control_setting_conf_.waitingtimelong()){
          controlflag.set_wait_flag(0);
          wait_time=0;
        }
      }else if(control_gear==1 && controlflag.wait_flag()!=1 ){
        //problem 这个分支能进来？
      }
      break;
    case 3: //启动模式
      controlcmd.set_control_steer_flag(1); //转向控制
      controlflag.set_park_flag(0);
      control_accpedal_flag=0;
      control_brkpedal_flag=1;
      control_clupedal_flag=1;
      control_brkpedal = 0;
      control_accpedal = 0;
      control_gear=1;
      delta_clu = control_setting_conf_.clutchreleaserate() * delta_t;
      if (control_clupedal > control_setting_conf_.clutchsetlow()) {
        control_clupedal = control_clupedal - delta_clu;          
      } else {
        control_clupedal = 0;
        if (vol_cur > vol_idle *0.8) {
          controlflag.set_start_flag(0);
        }
      }
      break;
    case 4: //停车模式
      controlcmd.set_control_steer_flag(1); //转向控制
      control_accpedal_flag=0;
      control_brkpedal_flag=1;
      control_clupedal_flag=1;
      control_accpedal = 0;
      control_clupedal = control_setting_conf_.clutchsethigh();
      control_gear = 1;
      delta_brk = control_setting_conf_.brakeapplyrate() * delta_t;
      if (control_brkpedal < control_setting_conf_.brakeset()) {
        control_brkpedal = control_brkpedal + delta_brk;        
      } else {
        control_brkpedal = control_setting_conf_.brakeset();
        if(vol_cur <= 0.5){
          controlflag.set_park_flag(1);
        }
      }
      break;
    case 5: //行驶模式
      controlcmd.set_control_steer_flag(1); //转向控制
      control_accpedal_flag=1;
      control_brkpedal_flag=0;
      control_clupedal_flag=0;
      control_brkpedal = 0;
      control_clupedal = 0;
      control_gear=1;
      control_accpedal = vol_exp/control_setting_conf_.kspeedthrottle()
            +(vol_exp-vol_cur)*control_setting_conf_.kdrive() ;
      break;
    case 6://制动模式
      controlcmd.set_control_steer_flag(1); //转向控制
      control_accpedal_flag = 0;
      control_brkpedal_flag = 1;
      control_clupedal_flag = 0;

      control_brkpedal = -(vol_exp - vol_cur) * control_setting_conf_.kbrake();
      control_accpedal = 0;
      control_clupedal = 0;
      break;
    default:
      break;
  }
  AINFO<< controlflag.DebugString();
  AINFO<<"vol="<< vol_cur;
  flag_writer->Write(controlflag);
}

