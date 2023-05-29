#ifndef _ALGORITHMAPI_
#define _ALGORITHMAPI_ 
//#include <filters/filter_base.hpp>
//#include <laser_filters/SpeckleFilterConfig.h>
#include <sensor_msgs/LaserScan.h>



bool checkWindowValid(const sensor_msgs::LaserScan& scan, size_t idx, size_t window, double max_range_difference);
bool checkWindowValid2(const sensor_msgs::LaserScan& scan, size_t idx, size_t window, double max_distance);

bool filter(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& output_scan
,int type,double max_range,double min_range,double max_range_difference,int filter_window);
#endif
