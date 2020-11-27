#include "guide_filter.h"

#include <string>
#include <iostream>
#include <cstdio>
using namespace std;

bool guide_Filter::Init() {


  chassis_detail_writer_ =
      node_->CreateWriter<ChassisDetail>("guide/ChassisDetail");
  // Create Writer

  chassis_detail_origin_reader_ = node_->CreateReader<ChassisDetail>(
      "guide/ChassisDetailOrig",
      [this](const std::shared_ptr<ChassisDetail>& msg) {UpdateChassisDetail(msg);});

  //read config;
  ReadConfig();


  AINFO<<"Filter Init Success";
  return true;
}

bool guide_Filter::Proc() {  // Timer callback;
  chassis_detail_writer_->Write(chassis_detail_);
  return true;
}
void guide_Filter::Clear() {  // shutdown

}
void guide_Filter::UpdateChassisDetail(const std::shared_ptr<ChassisDetail>& msg) {
  chassis_detail_=*msg;

  int FilterLength=configinfo.FilterLength;
  float UWBCorrect=configinfo.UWBCorrect;
  float LeaderBrakePedalCorrect=configinfo.LeaderBrakePedalCorrect;
  float LeaderAccPedalCorrect=configinfo.LeaderAccPedalCorrect;
  float SteerAngleCorrect=configinfo.SteerAngleCorrect;
  //uwb_azimuth 
  static AverageFilter<double> azimuth_filter(FilterLength, -180, 180);
  AINFO<<"Receive"<<msg->uwb_azimuth();
  azimuth_filter.AddElement(msg->uwb_azimuth()+ UWBCorrect);
  chassis_detail_.set_uwb_azimuth( azimuth_filter.GetValue());
  AINFO<<"Filter"<<azimuth_filter.GetValue();
  //uwb_distance
  static AverageFilter<double> distance_filter(FilterLength,0,100);
  distance_filter.AddElement(msg->uwb_distance() );
  chassis_detail_.set_uwb_distance( distance_filter.GetValue() );
  //steer_angle
  static AverageFilter<double> steer_angle_filter(FilterLength,-50000,50000);
  steer_angle_filter.AddElement(msg->steer_angle() + SteerAngleCorrect);
  chassis_detail_.set_steer_angle( steer_angle_filter.GetValue());
  //follower_speed
  static AverageFilter<double> follower_speed_filter(FilterLength,0,7200);
  follower_speed_filter.AddElement(msg->x_speed());
  chassis_detail_.set_x_speed( follower_speed_filter.GetValue());
  //Follower Yaw rate
  static AverageFilter<double> follower_yaw_rate_filter(FilterLength,-180,180);
  follower_yaw_rate_filter.AddElement(msg->follower_yaw_rate());
  chassis_detail_.set_follower_yaw_rate( follower_yaw_rate_filter.GetValue());

  static AverageFilter<double> leader_acc_pedal_filter(FilterLength,0,100);
  leader_acc_pedal_filter.AddElement(msg->leader_acc_pedal() + LeaderAccPedalCorrect);
  chassis_detail_.set_leader_acc_pedal( leader_acc_pedal_filter.GetValue());

  static AverageFilter<double> leader_brake_pedal_filter(FilterLength,0,100);
  leader_brake_pedal_filter.AddElement(msg->leader_brake_pedal() + LeaderBrakePedalCorrect);
  chassis_detail_.set_leader_brake_pedal( leader_brake_pedal_filter.GetValue());
  return;
}

void guide_Filter::ReadConfig(){
  ifstream f;
  f.open("/apollo/modules/guide_filter/Filter.config");
  if(f.is_open()) {
    string SettingName;
    while (!f.eof()) {
      f>>SettingName;
      if(SettingName == "FilterLength"){
        f>>configinfo.FilterLength;
        AINFO<<"FilterLength="<<configinfo.FilterLength;
      }else if(SettingName == "UWBCorrect"){
        f>>configinfo.UWBCorrect;
        AINFO<<"UWBCorrect="<<configinfo.UWBCorrect;
      }else if(SettingName == "LeaderBrakePedalCorrect"){
        f>>configinfo.LeaderBrakePedalCorrect;
        AINFO<<"LeaderBrakePedalCorrect="<<configinfo.LeaderBrakePedalCorrect;
      }else if(SettingName == "LeaderAccPedalCorrect"){
        f>>configinfo.LeaderAccPedalCorrect;
        AINFO<<"LeaderAccPedalCorrect="<<configinfo.LeaderAccPedalCorrect;
      }else if(SettingName == "SteerAngleCorrect"){
        f>>configinfo.SteerAngleCorrect;
        AINFO<<"SteerAngleCorrect="<<configinfo.SteerAngleCorrect;
      }
    }
    f.close();
    AINFO<<"ConfigFile Read ok";
  }
  else AERROR << "Filter.config Missing";

}