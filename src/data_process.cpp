#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include "data_process.h"

void get_data(const sensor_msgs::LaserScan& scan, int idx, double& dist, double& energy)
{
    if (idx < 0) 
        idx += scan.ranges.size();
    else if (idx >= scan.ranges.size()) 
        idx -= scan.ranges.size();
    dist = scan.ranges[idx];
    energy = scan.intensities[idx];
}

bool is_high(const sensor_msgs::LaserScan& scan, int idx)
{
    double dist, energy;
    get_data(scan, idx, dist, energy);

    return dist < 1000000 && energy > 200;
}

bool data_process(sensor_msgs::LaserScan& scan)
{
    int idx = 0;
    while (idx < scan.ranges.size())
    {
        if (scan.ranges[idx] < 10000000)
        {
            idx++;
            continue;
        }

        bool bl1 = is_high(scan, idx-1);
        bool bl2 = is_high(scan, idx-2);
        bool br1 = is_high(scan, idx+1);
        bool br2 = is_high(scan, idx+2);
        bool br3 = is_high(scan, idx+3);

        if ( (bl1 && br1 && br2) || (bl1 && bl2 && br1) )
        {
            double d1, d2, e1, e2;
            get_data(scan, idx-1, d1, e1);
            get_data(scan, idx+1, d2, e2);
            scan.ranges[idx] = (d1+d2)/2;
            scan.intensities[idx] = (e1+e2)/2;
            idx+=2;
        }
        else if (bl1 && bl2 && !br1 && br2 && br3)
        {
            double d1, d2, e1, e2;
            get_data(scan, idx-1, d1, e1);
            get_data(scan, idx+2, d2, e2);
            scan.ranges[idx] = (d1+d2)/2;
            scan.intensities[idx] = (e1+e2)/2;
            scan.ranges[idx+1] = (d1+d2)/2;
            scan.intensities[idx+1] = (e1+e2)/2;
            idx+=2;
        }
        else {
            idx++;
        }
        
    }
    return true;
}