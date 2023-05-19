#ifndef __DATA_PROC__
#define __DATA_PROC__
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include<vector>
#include"parser.h"
typedef struct
{
    unsigned short N;
    uint32_t ts[2];

    DataPoint points[0];

}DataPoints,*PDataPoints;

bool data_process(sensor_msgs::LaserScan&);
bool whole_data_process(RawData raw, bool from_zero,int collect_angle,std::vector<RawData>& whole_datas,PDataPoints *data,int &size,char *result);
int autoGetFirstAngle(RawData raw, bool from_zero, std::vector<RawData> &raws,std::string &result);
int find(std::vector<RawData>a, int n, int x);
#endif