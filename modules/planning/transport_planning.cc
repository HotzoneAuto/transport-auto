#include "transport_planning.h"

bool transport_Planning::Init() {
  AINFO << "Transport_Control init";
  if (!GetProtoConfig(&planning_setting_conf_)) {
    AERROR << "Unable to load conf file" << ConfigFilePath();
    return false;
  }
  if (!planning_setting_conf_.trajmode()) {
    file_csv.open_file(fname);
    std::string msg_w = "frame,pgsnh,gpsnl,gpseh,gpsel,heading_angle,"
                        "yaw_rate,gps_state,gps_velocity,acceleration_forward,"
                        "acceleration_lateral,acceleration_down,pitch_angle,"
                        "velocity_down,velocity_lateral,velocity_forward,roll_angle";
    file_csv.write_file(msg_w);
    frame = 0;
  } else {
    rel_loc_writer = node_->CreateWriter<apollo::planning::RelLoc>("/transport/planning");
    ReadTraj();
  }
  return true;
}

void transport_Planning::ReadTraj() {
  //读取所有点的经纬度
  traj_record_file.open(fname, std::ios::in);
  char linestr[500] = {0};
	traj_record_file.getline(linestr,500);
	while (traj_record_file.getline(linestr, 500)) {
		std::stringstream ss(linestr);
		std::string csvdata[17];
		double gpsnh,gpsnl,gpseh,gpsel,velocity;
		for (int i = 0; i < 17; i++) {
			char tempdata[500] = {0};
			ss.getline(tempdata,500,',');
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


void transport_Planning::UpdateTraj(const std::shared_ptr<Gps>& msg0) {
  //将靠近的若干个点转移到车辆的坐标系中
  //寻找轨迹上一段范围内的距离本车最近的点
  double N_now = msg0->gpsnl() + msg0->gpsnh();
  double E_now = msg0->gpsel() + msg0->gpseh();
  double Azi_now = msg0->heading_angle() / 180 * M_PI;
  int lastindex = TrajIndex;
  double min_dis = MAXDIS;
  for (int i = lastindex;
       i < std::min(lastindex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[3][i] + trajinfo[4][i];
    double dis = apollo::drivers::gps::SphereDis(E_now, N_now, E_point, N_point);
    if (dis < min_dis) {
      min_dis = dis;
      TrajIndex = i;
    }
  }
  //将该点附近的若干个点加入到自车坐标系中
  auto msg_rel_loc = std::make_shared<apollo::planning::RelLoc>();
  for (int i = TrajIndex;
       i < std::min(TrajIndex + TRAJLENGTH, (int)trajinfo[0].size()); i++) {
    double N_point = trajinfo[0][i] + trajinfo[1][i];
    double E_point = trajinfo[2][i] + trajinfo[3][i];
    double dis = apollo::drivers::gps::SphereDis(E_now, N_now, E_point, N_point);
    double azi = apollo::drivers::gps::SphereAzimuth(E_now, N_now, E_point, N_point);
    double rel_x = dis * cos(azi - Azi_now);
    double rel_y = dis * sin(azi - Azi_now);
    double vel = trajinfo[4][i];
    auto* rel_loc_position = msg_rel_loc->add_rel_loc_position();
    rel_loc_position->set_rel_x(rel_x);
    rel_loc_position->set_rel_y(rel_y);
    rel_loc_position->set_rel_vel(vel);
  }
  rel_loc_writer->Write(msg_rel_loc);
}

/*
  Reader Callback function
*/
bool transport_Planning::Proc(const std::shared_ptr<Gps>& msg0) {
  if (!planning_setting_conf_.trajmode()) {
    if (frame == 65535) {
      frame = 0;
    }
    frame ++;

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
                    + "," + std::to_string(msg0->roll_angle());
    file_csv.write_file(msg_w);
  }

  UpdateTraj(msg0);
  return true;
}

void transport_Planning::Clear() {
  if (!planning_setting_conf_.trajmode()) {
    file_csv.close_file();
  }
}
