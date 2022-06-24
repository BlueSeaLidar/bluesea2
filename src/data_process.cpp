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

        bool b = true;
        for (int i=0; i<2 && b; i++)
        {
            if ( !is_high(scan, idx-i) )
                b = false;
            else if (!is_high(scan, idx+1) )
                b = false;
        }
        if (!b) {
            idx++;
            continue;
        }
        double d1, d2, e1, e2;
        get_data(scan, idx-1, d1, e1);
        get_data(scan, idx+1, d2, e2);

        scan.ranges[idx] = (d1+d2)/2;
        scan.intensities[idx] = (e1+e2)/2;

        idx += 2;
    }
    return true;
}