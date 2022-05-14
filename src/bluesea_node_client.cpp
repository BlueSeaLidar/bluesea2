#include "ros/ros.h"
//#include"std_srvs/Empty.h"
#include<bluesea2/Control.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluesea2_node_client");
  if (argc != 3)
  {
    ROS_INFO("usage: rosrun bluesea2  bluesea2_node_client start  1     arg1 is start/stop   arg2 is num");
    return 1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client;
  char cmd[64] = {0};
  sprintf(cmd,"/lidar1/%s_motor",argv[1]);
  ROS_INFO("communicate service:%s\n",cmd);
  client = n.serviceClient<bluesea2::Control>(cmd);
  bluesea2::Control  srv;
  srv.request.index=atoi(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("OK");

  }
  else
  {
    ROS_ERROR("Failed to call service:%s",cmd);
    return 1;
  }

  return 0;
}