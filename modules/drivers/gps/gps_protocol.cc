#include "gps_protocol.h"
#include <iostream>

namespace apollo {
namespace drivers {
namespace gps {

GPSinfo::GPSinfo() {
};

bool GPSinfo::GetGPSinfo(char *s) {
  GPSString = s;
  return GetGPSinfo(GPSString);
}
bool GPSinfo::GetGPSinfo(std::string s) {
  if (s == "") return false;
  GPSString = s;
  int index = GPSString.find(";", 0);
  std::string Head, Info;
  Head = GPSString.substr(0, index);
  // cout << Head << endl;
  Info = GPSString.substr(index + 1) + ",";
  // cout << Info << endl;
  int position = 0;
  int count = 2;
  std::string GPSsubstr[26];
  while (Info.find(",", position) != std::string::npos) {
    int temppos = Info.find(',', position);
    GPSsubstr[count] = Info.substr(position, temppos - position);
    position = temppos + 1;
    count++;
  }
  index = GPSsubstr[13].find("*", 0);
  GPSsubstr[14] = GPSsubstr[13].substr(index + 1);
  GPSsubstr[13] = GPSsubstr[13].substr(0, index);
  // for(int i=2;i<=count;i++)	cout << GPSsubstr[i] << endl;
  Longitude = atof(GPSsubstr[5].data());
  Latitude = atof(GPSsubstr[4].data());
  NorthSpeed = atof(GPSsubstr[7].data());
  EastSpeed = atof(GPSsubstr[8].data());
  Azimuth = atof(GPSsubstr[12].data());
  Status = GetGPSstatus(GPSsubstr[13]);
  if (Status == GPSstatus::SOLUTION_GOOD)
    Ok = true;
  else
    Ok = false;
  return true;
}

// TODO(fengzongbao)
// warning: control reaches end of non-void function [-Wreturn-type]
GPSstatus GPSinfo::GetGPSstatus(std::string s) {
  if (s == "INS_INACTIVE")
    return GPSstatus::INACTIVE;
  else if (s == "INS_ALIGNING")
    return GPSstatus::ALIGNING;
  else if (s == "INS_HIGH_VARIANCE")
    return GPSstatus::HIGH_VARIANCE;
  else if (s == "INS_SOLUTION_GOOD")
    return GPSstatus::SOLUTION_GOOD;
  else if (s == "INS_SOLUTION_FREE")
    return GPSstatus::SOLUTION_FREE;
  else if (s == "INS_ALIGNMENT_COMPLETE")
    return GPSstatus::ALIGNMENT_COMPLETE;
  else if (s == "DETERMINING_ORIENTATION")
    return GPSstatus::DETERMINING_ORIENTATION;
  else if (s == "WAITING_INITIALPOS")
    return GPSstatus::WAITING_INITIALPOS;
}
}  // namespace gps
}  // namespace drivers
}  // namespace apollo