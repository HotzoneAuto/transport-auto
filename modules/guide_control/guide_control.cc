#include "guide_control.h"
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
//#define ACC_LIMIT 0.5
//#define DEACC_LIMIT -4

float Desired_speed = 0;
// float Desired_distance = 20;
constexpr float L = 3.975;
constexpr float r = 1.0;
constexpr int BezierLength=251;
// constexpr float k_a = 0.1;
// constexpr float k_v = 0.05;
// constexpr float k_d = 0.05;

bool guide_Control::Init() {
  using namespace std;
  AINFO << "Guide_Control init";
  ReadConfig();
  writer = node_->CreateWriter<ControlCommand>("guide/ControlCommand");
  // Init ControlCommand Writer
  return true;
}

/*
  Reader Callback function
*/

bool guide_Control::Proc(const std::shared_ptr<ChassisDetail>& msg0,
                         const std::shared_ptr<TrajInfo>& msg1) {
  
  // calculate steer
  float distance = msg0->uwb_distance();
  float azimuth = msg0->uwb_azimuth();
  AINFO << "reveiced Chassis Detail UWBdistance:" << distance
        << "  azimuth:" << azimuth;
  float control_steer=0;
  if(msg1->rel_x_size()>0){
    DealTraj(msg1);
    control_steer = Caculate_steer(msg0, msg1);
  }
  else{
    control_steer = Caculate_steer_purefollow(msg0);
    AERROR<<"Traj missing";
  }
  
  controlcmd.set_control_steer(control_steer - configinfo.ControlSteerCorrect); //Correct ControlSteerAngle

  // calculate acc
  control_acc = Caculate_acc(msg0);
  controlcmd.set_control_acc(control_acc);
  AINFO << controlcmd.DebugString();

  writer->Write(controlcmd);
  return true;
}

void guide_Control::DealTraj(const std::shared_ptr<TrajInfo>& msg1) {
  using namespace std;
  // save Traj
  int number = msg1->rel_x_size();
  FILE* f;
  f = fopen("/apollo/modules/traj.record", "w");
  if (f != NULL) {
    fprintf(f, "%d\n", number);
    for (int i = 0; i < number; i++) {
      fprintf(f, "%f %f\n", msg1->rel_x(i), msg1->rel_y(i));
    }
    fclose(f);
  }

  // Bezier Curve
  BezierFitting(msg1);
  // Lateral error
  int index_la = FindLookAheadPointBezier(6); //Todo 此处需要增加车身长度的修改量。
  err_lat = BezierX[index_la];
  AINFO << "Lateral Error is: " << err_lat;
}

/* Find Bezier Curve.*/

void guide_Control::BezierFitting(const std::shared_ptr<TrajInfo>& msg1) {
  // double BezierX[251], BezierY[251];//TODO: points number
  for (int i = 0; i <= 250; i++) BezierX[i] = BezierY[i] = 0;
  int number = msg1->rel_x_size();
  // 4 control point
  int ControlPointIndex[6];
  float Px[6], Py[6];
  for (int i = 0; i < 6; i++) {
    ControlPointIndex[i] = floor((number - 1) * i / 5);
    Px[i] = msg1->rel_x(ControlPointIndex[i]);
    Py[i] = msg1->rel_y(ControlPointIndex[i]);
  }
  // Cal Curve
  for (int i = 0; i < 251; i++) {
    BezierX[i] = CalBezierLoc(6, i * 0.004, Px);
    BezierY[i] = CalBezierLoc(6, i * 0.004, Py);
  }
  FILE* f;
  f = fopen("/apollo/modules/bezier.record", "w");
  if (f != NULL) {
    fprintf(f, "%d\n", 251);
    for (int i = 0; i < 251; i++) {
      fprintf(f, "%f %f\n", BezierX[i], BezierY[i]);
    }
    fclose(f);
  }
}

float guide_Control::CalBezierLoc(int n, float t, float p[]) {
  float loc = 0;
  // X(t)=sigma( nchoosek(n-1,k)*t^k*(1-t)^(n-1-k)*Px)
  for (int j = 0; j < n; j++) {
    float ret = 1;
    for (int i = 0; i < j; i++) ret *= t;
    for (int i = 0; i < n - 1 - j; i++) ret *= (1 - t);
    // nchoosek
    for (int i = n - 1; i >= n - j; i--) ret *= (i);
    for (int i = 1; i <= j; i++) ret /= i;
    ret *= p[j];
    loc += ret;
  }
  return loc;
}

float guide_Control::CalculateUnstableCoefficient(){
  float KErrY=configinfo.KErrY;
  float KErrTheta=configinfo.KErrTheta;
  float ErrY2Eff=configinfo.ErrY2Eff;
  float ErrTheta2Eff=configinfo.ErrTheta2Eff;


  float ErrorY=abs(BezierY[0]);
  float ErrorTheta=0;
  //CalculateErrorTheta by slope     degree
  if(BezierX[1]!=0){
    ErrorTheta=abs( atan( (BezierY[1]-BezierY[0])/(BezierX[1]-BezierX[0]) ) /M_PI*180);
  }
  float Du=0;
  if(ErrorY>ErrY2Eff) Du=Du+KErrY*(ErrorY-ErrY2Eff);
  if(ErrorTheta>ErrTheta2Eff) Du=Du+KErrTheta*(ErrorTheta-ErrTheta2Eff);
  if(Du>1) Du=1;
  AINFO<< "ErrorY = "<< ErrorY <<" ErrorTheta = " << ErrorTheta;
  AINFO<< "UnstableCoefficient = "<< Du;
  return Du;
}

float guide_Control::Stanley(float k,float v,int &ValidCheck){ // use stanley method to calculate steer angle.
  ValidCheck=1;
// find point X=L
  int index=BezierLength-1;
  for(int i=0;i<BezierLength-1;i++){
    if(BezierX[i]<=L && BezierX[i+1]>L){
      index=i;
      break;
    }
  }
  //maybe no such point?
  if(BezierX[index]>L || BezierX[index+1]<=L) {
    ValidCheck=0;
    return 0;
  }
// find angle at point X=L 
  float phi_e=atan( (BezierY[index+1]-BezierY[index]) / (BezierX[index+1]-BezierX[index]))/M_PI*180;
  //X[i] and X[i+1] may be same?

// find y error at point X=L
  if(v<=3) v=3;
  float phi_y=atan( k*( BezierY[index] + (BezierY[index+1]-BezierY[index]) *
                           (L-BezierX[index]) / (BezierX[index+1]-BezierX[index]) )/v );
  //v==0?
  return phi_e+phi_y;
}

//纯跟踪预瞄
float guide_Control::Caculate_steer_purefollow(const std::shared_ptr<ChassisDetail>& msg0) {

  // get config in current speed
  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();
  if (distance == 0) return 4.8505;
  float long_distance;
  long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // m
  float lat_distance;
  lat_distance = distance * sin(azimuth_angle / 180 * M_PI);  // m

  AINFO << "purefollow x is: " << long_distance;
  AINFO << "purefollow y is: " << lat_distance;

  float frontwheel_steer_angle =
      atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 /
          M_PI;
  if (frontwheel_steer_angle > 20)
    frontwheel_steer_angle = 20;
  else if (frontwheel_steer_angle < -20)
    frontwheel_steer_angle = -20;  // steer angle limit
  // cout << "Frontwheel_steer_angle = "<< to_string(frontwheel_steer_angle) <<
  // endl;

  // Saturation
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_steer degree
*/

float guide_Control::Caculate_steer(const std::shared_ptr<ChassisDetail>& msg0,
                                    const std::shared_ptr<TrajInfo>& msg1) {
  //Read ConfigInfo
  float CurrentSpeed = msg0->x_speed();
  float LookAheadDistance = 0;
  float steerK, steer_stanley_prop;
  float LatAccLimit=configinfo.LatAccLimit;
  int SpeedStage = 0;
  if (CurrentSpeed <= configinfo.Speed[1])
    SpeedStage = 0;
  else if (CurrentSpeed <= configinfo.Speed[2])
    SpeedStage = 1;
  else
    SpeedStage = 2;

  LookAheadDistance = configinfo.LookAheadDistance[SpeedStage];
  steerK = configinfo.SteerK;

  steer_stanley_prop = configinfo.SteerStanleyProportion;
  // get config in current speed

  //Find LookAhead Point in Bezier Curve
  int index_la = FindLookAheadPointBezier(LookAheadDistance);
  float long_distance;
  long_distance = BezierX[index_la];
  float lat_distance;
  lat_distance = BezierY[index_la];

  AINFO << "lookahead x is: " << long_distance;
  AINFO << "lookahead y is: " << lat_distance;
  //Calculate Pid Control FrontWheel Angle
  //PID pid_steer(steer_PID_kp, steer_PID_ki, steer_PID_kd);  // todo delete PID Class
  int ValidCheck=0;
  float stanley_angle=-Stanley(steerK,CurrentSpeed,ValidCheck);   //stanley method
  if(ValidCheck==0){//invalid then only use follow
    steer_stanley_prop=0;
  }
  float frontwheel_steer_angle1 =
      -(1 - steer_stanley_prop) *
          atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 /
          M_PI +
      steer_stanley_prop * stanley_angle;
  
  //Calculate FrontWheelAngle Pure Follow
  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();

  long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // m
  lat_distance = distance * sin(azimuth_angle / 180 * M_PI);  // m
  float frontwheel_steer_angle2 =0;
  if (long_distance == 0) frontwheel_steer_angle2 = 0;
  else  frontwheel_steer_angle2 = atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 / M_PI;

  //sum frontwheel angle from 2 sources
  float Du=CalculateUnstableCoefficient();
  float frontwheel_steer_angle = frontwheel_steer_angle1*(1-Du) + frontwheel_steer_angle2*Du;

  //Calculate Lateral Acc Limit
  float frontwheel_angle_limit;
  frontwheel_angle_limit = atan(LatAccLimit*L/CurrentSpeed/CurrentSpeed)/M_PI*180;
  if(frontwheel_angle_limit < 0){
      frontwheel_angle_limit = -frontwheel_angle_limit;
  }else if(frontwheel_angle_limit > 20){
      frontwheel_angle_limit=20;
  }
  
  if (frontwheel_steer_angle > frontwheel_angle_limit)
    frontwheel_steer_angle = frontwheel_angle_limit;
  else if (frontwheel_steer_angle < -frontwheel_angle_limit)
    frontwheel_steer_angle = -frontwheel_angle_limit;  // steer angle limit
  // cout << "Frontwheel_steer_angle = "<< to_string(frontwheel_steer_angle) <<
  // endl;

  // Saturation
  float steer_wheel_angle =
      24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_acc m/s^2
*/
float guide_Control::Caculate_acc(const std::shared_ptr<ChassisDetail>& msg0) {
  float v1 = msg0->leader_speed();
  float v2 = msg0->x_speed();
  float a1 = msg0->leader_acc();
  float distance = msg0->uwb_distance();
  float azimuth_angle = msg0->uwb_azimuth();
  float long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // m
  float Leader_Brake_pedal = msg0->leader_brake_pedal();
  float Leader_Acc_pedal = msg0->leader_acc_pedal();
  float control_acc = 0;
  float Desired_distance = configinfo.DesiredDistance;
  // read from message

  float CurrentSpeed = msg0->x_speed();
  float k_a, k_v, k_d;
  float ACC_LIMIT = configinfo.AccLimit[1];
  float DEACC_LIMIT = configinfo.AccLimit[0];
  int SpeedStage = 0;
  float DeltaS_down, DeltaS_up;
  float PedalEffect, AccCorrectMax, BrakeCorrectMax;
  if (CurrentSpeed <= configinfo.Speed[1])
    SpeedStage = 0;
  else if (CurrentSpeed <= configinfo.Speed[2])
    SpeedStage = 1;
  else
    SpeedStage = 2;
  k_a = configinfo.AccKa[SpeedStage];
  k_v = configinfo.AccKv[SpeedStage];
  k_d = configinfo.AccKd[SpeedStage];
  DeltaS_down = configinfo.DeltaS[0];
  DeltaS_up = configinfo.DeltaS[1];
  PedalEffect = configinfo.PedalEffect;
  AccCorrectMax = configinfo.AccCorrectMax;
  BrakeCorrectMax = configinfo.BrakeCorrectMax;
  // // get config in current speed

  /******Distance Keeping Control*****/

  // cout << "a = " << a1 << endl;
  // cout << "delta v = " << v1-v2 << endl;
  // cout << "delta x = " << long_distance - Desired_distance << endl;
  float distance_error = distance - float(Desired_distance);
  AINFO << "distance_error= " << distance_error;
  AINFO << "Leader_Brake_pedal=" << Leader_Brake_pedal << " "
        << "Leader_Acc_pedal=" << Leader_Acc_pedal;
  if (distance_error < DeltaS_up && distance_error > DeltaS_down)
    control_acc = k_a * a1 + k_v * (v1 - v2) + k_d * distance_error;
  else
    control_acc = k_a * a1 / 2 + k_v * (v1 - v2) / 2 + k_d * distance_error * 2;

  if (control_acc < 0) control_acc = control_acc * 4;
  // cout << "Leader Brake Pedal = " << Leader_Brake_pedal << endl;

  if (Leader_Brake_pedal > 0) {
    double pedal = Leader_Brake_pedal / 100;
    if (pedal > PedalEffect) {
      float kBrake = configinfo.kBrake;
      float bBrake = configinfo.bBrake;
      double k = kBrake * pedal + bBrake;
      if (k > BrakeCorrectMax) k = BrakeCorrectMax;

      AINFO << "AccCorrect Value = -" << k * pedal;
      control_acc = control_acc - k * pedal;
    }
  } else if (Leader_Acc_pedal > 0) {
    double pedal = Leader_Acc_pedal / 100;
    if (pedal > PedalEffect) {
      float kAcc = configinfo.kAcc;
      float bAcc = configinfo.bAcc;
      double k = kAcc * pedal + bAcc;
      if (k > AccCorrectMax) k = AccCorrectMax;
      AINFO << "AccCorrect Value = " << k * pedal;
      control_acc = control_acc + k * pedal;
    }
  }
  // Acc Correct by Leader pedal

  // Saturation
  if (control_acc > ACC_LIMIT)
    control_acc = ACC_LIMIT;  // acc limit
  else if (control_acc < DEACC_LIMIT)
    control_acc = DEACC_LIMIT;  // deacc limit

  AINFO << "contorl_acc= " << control_acc;
  return control_acc;
}
//no use
int guide_Control::FindLookAheadPoint(float LookAheadDis,
                                      const std::shared_ptr<TrajInfo>& msg1) {
  int index_max = 0;  // The possible max index of LA point
  while (msg1->rel_x(index_max) < LookAheadDis) {
    index_max++;
  }

  int index_la = index_max;  // The index of LA point
  while (msg1->rel_x(index_la) * msg1->rel_x(index_la) +
             msg1->rel_y(index_la) * msg1->rel_y(index_la) -
             LookAheadDis * LookAheadDis >
         0) {
    index_la--;
  }
  return (index_la);
}
/******************************************************************
 * Function: FindLookAheadPoint;
 * Description: Find the index of the point nearest of the LA point,
 * and return the index;
 * Input:
 *      float LookAheadDis: Lookahead Distance;
 *      const std::shared_ptr<TrajInfo>& msg1: the pointer to the
 *                    trajectory infomation;
 * Output:
 *      int index_la: the index of the la point;
 *****************************************************************/

int guide_Control::FindLookAheadPointBezier(float LookAheadDis) {
  float DisSum = 0;
  int i;
  for (i = 1; i < 251; i++) {
    float dis =
        sqrt((BezierX[i] - BezierX[i - 1]) * (BezierX[i] - BezierX[i - 1]) +
             (BezierY[i] - BezierY[i - 1]) * (BezierY[i] - BezierY[i - 1]));
    DisSum += dis;
    if (DisSum > LookAheadDis) break;
  }
  return i;
}

void guide_Control::ReadConfig() {
  using namespace std;
  // map<string,float> configmap;
  ifstream f;
  f.open("/apollo/modules/guide_control/ControlSettings.config");
  if (f.is_open()) {
    AINFO << "Control Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      ConfigInfo& x = configinfo;
      if (SettingName == "Speed") {
        f >> x.Speed[0] >> x.Speed[1] >> x.Speed[2];
        x.Speed[0] /= 3.6;
        x.Speed[1] /= 3.6;
        x.Speed[2] /= 3.6;
      } else if (SettingName == "DesiredDistance") {
        f >> x.DesiredDistance;
      } /*else if (SettingName == "SteerKp") {
        f >> x.SteerKp[0] >> x.SteerKp[1] >> x.SteerKp[2];
      } else if (SettingName == "SteerKi") {
        f >> x.SteerKi[0] >> x.SteerKi[1] >> x.SteerKi[2];
      } else if (SettingName == "SteerKd") {
        f >> x.SteerKd[0] >> x.SteerKd[1] >> x.SteerKd[2];
      } */
      else if(SettingName == "SteerK"){
        f>>x.SteerK;
      }else if (SettingName == "LookAheadDistance") {
        f >> x.LookAheadDistance[0] >> x.LookAheadDistance[1] >>
            x.LookAheadDistance[2];
      } else if (SettingName == "SteerStanleyProportion") {
        f >> x.SteerStanleyProportion;
      } else if (SettingName == "AccKd") {
        f >> x.AccKd[0] >> x.AccKd[1] >> x.AccKd[2];
      } else if (SettingName == "AccKv") {
        f >> x.AccKv[0] >> x.AccKv[1] >> x.AccKv[2];
      } else if (SettingName == "AccKa") {
        f >> x.AccKa[0] >> x.AccKa[1] >> x.AccKa[2];
      } else if (SettingName == "DeltaS") {
        f >> x.DeltaS[0] >> x.DeltaS[1];
        if (x.DeltaS[0] >= x.DeltaS[1]) {
          float temp = x.DeltaS[0];
          x.DeltaS[0] = x.DeltaS[1];
          x.DeltaS[1] = temp;
        }
      } else if (SettingName == "AccLimit") {
        f >> x.AccLimit[0] >> x.AccLimit[1];
        if (x.AccLimit[0] >= x.AccLimit[1]) {
          float temp = x.AccLimit[0];
          x.AccLimit[0] = x.AccLimit[1];
          x.AccLimit[1] = temp;
        }
      } else if (SettingName == "PedalEffect") {
        f >> x.PedalEffect;
      } else if (SettingName == "kAcc") {
        f >> x.kAcc;
      } else if (SettingName == "kBrake") {
        f >> x.kBrake;
      } else if (SettingName == "bAcc") {
        f >> x.bAcc;
      } else if (SettingName == "bBrake") {
        f >> x.bBrake;
      } else if (SettingName == "AccCorrectMax") {
        f >> x.AccCorrectMax;
      } else if (SettingName == "BrakeCorrectMax") {
        f >> x.BrakeCorrectMax;
      } else if (SettingName == "ControlSteerCorrect"){
        f>>x.ControlSteerCorrect;
      } else if(SettingName == "LatAccLimit"){
        f>>x.LatAccLimit;
        AINFO<<"LatAccLimit= " << x.LatAccLimit;
      } else if(SettingName == "KErrY"){
        f>>x.KErrY;
        AINFO<<"KErrY= " << x.KErrY;
      } else if(SettingName == "KErrTheta"){
        f>>x.KErrTheta;
        AINFO<<"KErrTheta= " << x.KErrTheta;
      } else if(SettingName == "ErrY2Eff"){
        f>>x.ErrY2Eff;
        AINFO<<"ErrY2Eff= " << x.ErrY2Eff;
      } else if(SettingName == "ErrTheta2Eff"){
        f>>x.ErrTheta2Eff;
        AINFO<<"ErrTheta2Eff= " << x.ErrTheta2Eff;
      }
    }
    f.close();
  } else
    AERROR << "ControlSettings.config Missing";
  // output Configinfo
  AINFO << "Config Parameters:";
  AINFO << "Desireddistance" << configinfo.DesiredDistance;
  AINFO << "Speed " << configinfo.Speed[0] << " " << configinfo.Speed[1] << " "
        << configinfo.Speed[2];
        /*
  AINFO << "SteerKp " << configinfo.SteerKp[0] << " " << configinfo.SteerKp[1]
        << " " << configinfo.SteerKp[2];
  AINFO << "SteerKi " << configinfo.SteerKi[0] << " " << configinfo.SteerKi[1]
        << " " << configinfo.SteerKi[2];
  AINFO << "SteerKc " << configinfo.SteerKd[0] << " " << configinfo.SteerKd[1]
        << " " << configinfo.SteerKd[2];*/
  AINFO << "SteerK" << configinfo.SteerK;
  AINFO << "LookAheadDistance" << configinfo.LookAheadDistance[0] << " "
        << configinfo.LookAheadDistance[1] << " "
        << configinfo.LookAheadDistance[2];
  AINFO << "SteerStanleyProportion" << configinfo.SteerStanleyProportion;
  AINFO << "AccKd " << configinfo.AccKd[0] << " " << configinfo.AccKd[1] << " "
        << configinfo.AccKd[2];
  AINFO << "AccKv " << configinfo.AccKv[0] << " " << configinfo.AccKv[1] << " "
        << configinfo.AccKv[2];
  AINFO << "AccKa " << configinfo.AccKa[0] << " " << configinfo.AccKa[1] << " "
        << configinfo.AccKa[2];
  AINFO << "DeltaS " << configinfo.DeltaS[0] << " " << configinfo.DeltaS[1];
  AINFO << "AccLimit " << configinfo.AccLimit[0] << " "
        << configinfo.AccLimit[1];
  AINFO << "PedalEffect" << configinfo.PedalEffect;
  AINFO << "kACC " << configinfo.kAcc;
  AINFO << "bACC " << configinfo.bAcc;
  AINFO << "kBrake " << configinfo.kBrake;
  AINFO << "bBrake " << configinfo.bBrake;
  AINFO << "AccCorrectMax " << configinfo.AccCorrectMax;
  AINFO << "BrakeCorrectMax " << configinfo.BrakeCorrectMax;
  AINFO << "ControlSteerCorrect" << configinfo.ControlSteerCorrect;
}
