#include "ros/ros.h"
//#include"std_srvs/Empty.h"
#include<bluesea2/Control.h>
#include<bluesea2/DefenceZone.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluesea2_node_client");
  ROS_INFO("%d\n",argc);

  if (argc != 3&&argc != 4)
  {
    ROS_INFO("usage: rosrun bluesea2  bluesea2_node_client start  1     arg1 is start/stop   arg2 is num");
    ROS_INFO("usage: rosrun bluesea2  bluesea2_node_client switchZone  1    192.168.0.110     arg1 is action   arg2 is zone arg3 is ip");
    return 1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client;
  char cmd[64] = {0};
  sprintf(cmd,"/lidar1/%s_motor",argv[1]);
  ROS_INFO("communicate service:%s\n",cmd);

  int result=0; 
  if(strcmp(argv[1],"start")==0||strcmp(argv[1],"stop")==0)
  {
    client = n.serviceClient<bluesea2::Control>(cmd);
    bluesea2::Control  srv;
    srv.request.index=atoi(argv[2]);
    result = client.call(srv);
  }
  else if(strcmp(argv[1],"switchZone")==0)
  {
    client = n.serviceClient<bluesea2::DefenceZone>(cmd);
    bluesea2::DefenceZone  srv;
    srv.request.index=atoi(argv[2]);
    srv.request.ip=argv[3];
    result = client.call(srv);
  }

  if (result)
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