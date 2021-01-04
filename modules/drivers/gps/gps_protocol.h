#pragma once

#include <cmath>
#include <cstring>
#include <string>

namespace apollo {
namespace drivers {
namespace gps {

const double PI = atan(1.0) * 4;

enum class GPSstatus {
  INACTIVE,
  ALIGNING,
  HIGH_VARIANCE,
  SOLUTION_GOOD,
  SOLUTION_FREE,
  ALIGNMENT_COMPLETE,
  DETERMINING_ORIENTATION,
  WAITING_INITIALPOS
};

class GPSinfo {
 public:
  GPSinfo();
  bool GetGPSinfo(char* s);
  bool GetGPSinfo(std::string s);
  GPSstatus GetGPSstatus(std::string s);

private:
  std::string GPSString = "";
  bool Ok = false;
  double Longitude = 0;
  double Latitude = 0;
  double NorthSpeed = 0;
  double EastSpeed = 0;
  double Azimuth = 0;
  GPSstatus Status;
};

double haversin(double theta) { return std::sin(theta / 2) * std::sin(theta / 2); }
double D2R(double theta) { return theta * PI / 180.0; }
double SphereDis(double lon1, double lat1, double lon2, double lat2) {
  // return  from p1 to p2 distance  unit:m
  double R = 6378.137 * 1000;
  double x1 = D2R(lon1), x2 = D2R(lat1);
  double y1 = D2R(lon2), y2 = D2R(lat2);
  double h =
      haversin(abs(x2 - y2)) + cos(x2) * cos(y2) * haversin(abs(x1 - y1));
  double d = 2 * R * asin(sqrt(h));
  return d;
}
double mod(double x, double y) {
  if (std::fmod(x, y) < 0) {
    return std::fmod(x, y) + y;
  } else
    return std::fmod(x, y);
}
double SphereAzimuth(double lon1, double lat1, double lon2, double lat2) {
  // return  from p1 to p2 azimuth  unit:rad
  double x1 = D2R(lon1), x2 = D2R(lat1);
  double y1 = D2R(lon2), y2 = D2R(lat2);
  double tc1 = mod(std::atan2(std::sin(y1 - x1) * std::cos(y2),
                         std::cos(x2) * std::sin(y2) - std::sin(x2) * std::cos(y2) * std::cos(y1 - x1)),
                   2 * PI);
  return tc1;
}
}  // namespace gps
}  // namespace drivers
}  // namespace apollo
