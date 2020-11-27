#include "Planner.h"
#include <fstream>
#include <string>
#define MAX_SEQV_SIZE 400
using namespace std;
void Planner::ReadConfig(){
  ifstream f;
  f.open("/apollo/modules/guide_planner/PlannerSettings.config");
  if (f.is_open()) {
    //AINFO << "Control Config File Opened";
    while (!f.eof()) {
      string SettingName;
      f >> SettingName;
      if (SettingName == "YawSourceProp") {
        f>>configinfo.YawSourceProp;
        cout<<configinfo.YawSourceProp;
      }
    }
    f.close();
  }
  //AERROR << "ControlSettings.config Missing";
}
bool Planner::Init(float rel_x, float rel_y, float rel_yaw) {
  ReadConfig();
  for (int i = 0; i < 5; i++) traj.Seqv[i].clear();
  for (int i = 0; i < PointsNumber; i++) {
    traj.Seqv[0].push_back(rel_x / (PointsNumber - 1) * i);
    traj.Seqv[1].push_back(rel_y / (PointsNumber - 1) * i);
    traj.Seqv[2].push_back(rel_yaw);  //相对转角
    traj.Seqv[3].push_back(3);
    traj.Seqv[4].push_back(0);
  }
  return isInit = true;
}
bool Planner::isinit() { return isInit; };

void Planner::traj_seq_update(float rel_x, float rel_y, float vx_1,
                              float yaw_rate_1, float last_vx_2,
                              float last_yaw_rate_2, float delta_f_2,
                              float rel_yaw) {
  // delta_f_2表示后车前轮转角
  //参数定义
  //float lf = 1.1, lr = 2.8;  //质心到前后轴距离
  float L=3.975;
  float delta_yaw=0;           //跟随车转动角度
  float YawSourceProp=configinfo.YawSourceProp;
  TrajSeq traj_seq;
  TrajSeq &last_traj_seq = traj;
  if (!isinit()) {
    return;
  } else {
    delta_yaw =
        last_yaw_rate_2 * SampleTime / 180 * Pi;  // Sampletime里转动的角度
    float R=0;
    float x_0=0;
    float y_0=0;
    if( abs( delta_f_2 ) > 0.2){ //当前轮转角较大时采用弧形模型计算
      R=L/tan(delta_f_2  * Pi / 180);
      float delta_yaw_2 = last_vx_2 * SampleTime / R; 
      cout<<"YawSourceProp:= "<<YawSourceProp<<"  delta_yaw:= "<<delta_yaw<<"  delta_yaw_2: "<<delta_yaw_2<<endl;
      delta_yaw = (1-YawSourceProp)*delta_yaw + YawSourceProp*delta_yaw_2; //采用2种方法融合加权计算
      
      cout<<"R="<<R<<"  delta_yaw: "<<delta_yaw <<endl;
      //x_0 =  abs( R * sin( delta_yaw) );  // Sampletime里x方向移动的距离
      x_0 = last_vx_2 * SampleTime;
      y_0 = R * (1-cos(delta_yaw));  // Sampletime里y方向移动的距离
    }
    else {
      x_0 = last_vx_2 * SampleTime;  // Sampletime里x方向移动的距离
      y_0 = 0;
      //y_0 = x_0 * lr / (lf + lr) *
         //       tan(delta_f_2 / 180 * Pi);  // Sampletime里y方向移动的距离
    }
    cout<<"x0: "<<x_0<<"  y0: "<<y_0<<endl;
    traj_seq = last_traj_seq;
    for (int i = 0; i < traj_seq.Seqv[0].size(); i++) {
      traj_seq.Seqv[0][i] = traj_seq.Seqv[0][i] - x_0;
      traj_seq.Seqv[1][i] = traj_seq.Seqv[1][i] - y_0;
      float tmp1 = cos(delta_yaw) * traj_seq.Seqv[0][i] +
                   sin(delta_yaw) * traj_seq.Seqv[1][i];
      float tmp2 = -sin(delta_yaw) * traj_seq.Seqv[0][i] +
                   cos(delta_yaw) * traj_seq.Seqv[1][i];  //使用坐标转换矩阵
      traj_seq.Seqv[0][i] = tmp1;
      traj_seq.Seqv[1][i] = tmp2;
      traj_seq.Seqv[2][i] =
          traj_seq.Seqv[2][i] - SampleTime * last_yaw_rate_2;  //更新yaw角
    }
    //剔除车后的轨迹点
    for (vector<float>::iterator it = traj_seq.Seqv[0].begin();
         it != traj_seq.Seqv[0].end();) {
      if (*it < 0 || traj_seq.Seqv[0].size() > MAX_SEQV_SIZE) {
        it = traj_seq.Seqv[0].erase(it);
        traj_seq.Seqv[1].erase(traj_seq.Seqv[1].begin());
        traj_seq.Seqv[2].erase(traj_seq.Seqv[2].begin());
        traj_seq.Seqv[3].erase(traj_seq.Seqv[3].begin());
        traj_seq.Seqv[4].erase(traj_seq.Seqv[4].begin());
      } else {
        break;
      }
    }
    cout<<"traj sz1=  :"<<traj_seq.Seqv[0].size()<<endl;
    //判断新点的距离
    float old_x = 0, old_y = 0;  //若没有点则默认为本车点
    if (traj_seq.Seqv[0].size() > 0) {
      old_x = traj_seq.Seqv[0][ traj_seq.Seqv[0].size() - 1];
      old_y = traj_seq.Seqv[1][ traj_seq.Seqv[0].size() - 1];
    }
    float dis = sqrt((old_x - rel_x) * (old_x - rel_x) +
                     (old_y - rel_y) * (old_y - rel_y));

    // to far then reset;
    if (dis > ErrorDistance) {
      // Init(rel_x,rel_y,rel_yaw);
      traj = traj_seq;
      return;
    }
    // 加入新轨迹点
    if (dis > PointDistance) {
      traj_seq.Seqv[0].push_back(rel_x);
      traj_seq.Seqv[1].push_back(rel_y);
      traj_seq.Seqv[2].push_back(rel_yaw);
      traj_seq.Seqv[3].push_back(vx_1);
      traj_seq.Seqv[4].push_back(yaw_rate_1);

    }
    traj = traj_seq;
  }
  //cout<<"traj szzz2=  :"<<traj_seq.Seqv[0].size();
}

// TrajSeq Planner::truth_seq_update(int timestep, TrajSeq last_truth_seq,
// float* abs_loc_1, float* abs_loc_2, float abs_yaw_1) {
// 	//abs_loc_1[2]引导车绝对坐标x,y   abs_loc_2[2]跟随车绝对坐标x，y
// abs_yaw_1前车绝对偏航角 	float abs_x_1 = abs_loc_1[0]; 	float abs_y_1 =
// abs_loc_1[1]; 	float abs_x_2 = abs_loc_2[0]; 	float abs_y_2 =
// abs_loc_2[1]; 	TrajSeq truth_seq = last_truth_seq; 	if (timestep ==
// 1) { 		for (int i = 0; i < PointsNumber; i++) {
// truth_seq.Seq[i][0] = abs_x_2 + (abs_x_1
// - abs_x_2) / (PointsNumber - 1) * i;
// truth_seq.Seq[i][1] = abs_y_2 + (abs_y_1 - abs_y_2) / (PointsNumber - 1) * i;
// truth_seq.Seq[i][2] = abs_yaw_1; 			truth_seq.Seq[i][3] = 0;
// 		}
// 	}
// 	else {
// 		for (int i = 0; i < PointsNumber - 1; i++)
// 			for (int j = 0; j < 5; j++) {
// 				truth_seq.Seq[i][j] = last_truth_seq.Seq[i +
// 1][j];
// 			}
// 		truth_seq.Seq[PointsNumber - 1][0] = abs_x_1;
// 		truth_seq.Seq[PointsNumber - 1][1] = abs_y_1;
// 		truth_seq.Seq[PointsNumber - 1][2] = abs_yaw_1;
// 		truth_seq.Seq[PointsNumber - 1][3] = 0;
// 	}
// 	return truth_seq;
// }

TrajSeq::TrajSeq() {
  number = 0;
  for (int i = 0; i < 5; i++) Seqv[i].clear();
}
void TrajSeq::Print() {
  for (int i = 0; i < Seqv[0].size(); i++) {
    cout << "point " << i << " :";
    for (int j = 0; j < 5; j++) cout << Seqv[j][i] << " ";
    cout << endl;
  }
  cout << endl;
}
