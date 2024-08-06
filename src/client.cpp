#include "ros/ros.h"
//#include"std_srvs/Empty.h"
#include<bluesea2/Control.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bluesea2_client");
  if (argc != 3&&argc != 4)
  {
    ROS_INFO("usage: rosrun bluesea2  bluesea2_client scan start                          arg1 is topic   arg2 is action(start/stop)");
    ROS_INFO("usage: rosrun bluesea2  bluesea2_client scan switchZone  0                  arg1 is topic   arg2 is action(switchZone)  arg3 is select zone id");
    ROS_INFO("usage: rosrun bluesea2  bluesea2_client scan rpm  600                       arg1 is topic   arg2 is action(set rpm)  arg3 is rpm value");
    ROS_INFO("usage: rosrun bluesea2  bluesea2_client heart check  1                      arg1 is service name   arg2 is action(check)  arg3 is print / not print");
    return 1;
  }

  ros::NodeHandle n;
  
  char cmd[64] = {0};
  sprintf(cmd,"/lidar1/%s_motor",argv[1]);
  ROS_INFO("communicate service node:%s\n",cmd);
  int result=0; 
  //ros::service::waitForService(cmd);
  ros::ServiceClient client=n.serviceClient<bluesea2::Control>(cmd);


  bluesea2::Control  srv;
  srv.request.topic=argv[1];
  srv.request.func=argv[2];
  if(strcmp(argv[2],"start")==0||strcmp(argv[2],"stop")==0)
  {
    //srv.request.flag=atoi(argv[3]);
  }
  else if(strcmp(argv[2],"switchZone")==0)
  {
    srv.request.flag=atoi(argv[3]);
    //srv.request.params=argv[4];
  }
  else if(strcmp(argv[2],"rpm")==0)
  {
    srv.request.params=argv[3];
    
  }
  else if(strcmp(argv[2],"check")==0)
  {
    // char tmpcmd[64] = {0};
    // sprintf(tmpcmd,"/%s_motor",argv[2]);
    // client = n.serviceClient<bluesea2::Control>(tmpcmd);

    srv.request.flag=atoi(argv[3]);
  }
  result = client.call(srv);
  if (result)
      ROS_INFO("code:%d value:%s",srv.response.code,srv.response.value.c_str());
  else
      ROS_ERROR("Failed to call service:%s",cmd);


  return 0;
}