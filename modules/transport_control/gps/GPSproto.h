#pragma once

#include <cstring>
#include <string>
#include <cmath>
using namespace std;
const double PI=atan(1.0)*4;
enum class GPSstatus
{
	INACTIVE, ALIGNING, HIGH_VARIANCE, SOLUTION_GOOD, SOLUTION_FREE, 
	ALIGNMENT_COMPLETE, DETERMINING_ORIENTATION, WAITING_INITIALPOS
};
class GPSinfo {
public:
	string GPSString;
	bool Ok;
	double Longitude;
	double Latitude;
	double NorthSpeed;
	double EastSpeed;
	double Azimuth;
	GPSstatus Status;
	GPSinfo();
	bool GetGPSinfo(char* s);
	bool GetGPSinfo(string s);
	GPSstatus GetGPSstatus(string s);
};
double haversin(double theta){return sin(theta/2)*sin(theta/2);}
double D2R(double theta){return theta*PI/180.0;}
double SphereDis(double lon1,double lat1,double lon2,double lat2){
	//return  from p1 to p2 distance  unit:m
	double R=6378.137*1000;
	double x1=D2R(lon1),x2=D2R(lat1);
	double y1=D2R(lon2),y2=D2R(lat2);
	double h=haversin(abs(x2-y2))+cos(x2)*cos(y2)*haversin(abs(x1-y1));
	double d=2*R*asin(sqrt(h));
	return d;
}
double mod(double x,double y){
	if(fmod(x,y)<0){ return fmod(x,y)+y;}
	else return fmod(x,y);
}
double SphereAzimuth(double lon1,double lat1,double lon2,double lat2){
	//return  from p1 to p2 azimuth  unit:rad
	double x1=D2R(lon1),x2=D2R(lat1);
	double y1=D2R(lon2),y2=D2R(lat2);
	double tc1=mod(atan2(sin(y1-x1)*cos(y2),cos(x2)*sin(y2)-sin(x2)*cos(y2)*cos(y1-x1)),2*PI);
	return tc1;
}
