
/*********************************************************************
 * Lanhai ROS driver
 * serial port version  LDS-25: 
 rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=1 _output_cloud:=0 _unit_is_mm:=0 _with_confidence:=0
 * serial port version  LDS-50: 
 rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=1 _output_cloud:=0 _unit_is_mm:=1 _with_confidence:=1
 * UDP network version like this:
 rosrun bluesea2 bluesea2_node _frame_id:=map _type:=udp _lidar_ip:=192.168.158.91 _firmware_version:=2 _output_scan:=1 _output_cloud:=1
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <bluesea2/DynParamsConfig.h>

#include <pthread.h>

#include "reader.h"

HReader g_reader = NULL;
std::string g_type = "uart";

struct PubHub
{
	pthread_mutex_t mtx;
	int nfan;
	RawData* fans[MAX_FANS];
};


bool SendCmd(int len, char* cmd)
{
	if ( g_type == "uart" ) 
	{
		return SendUartCmd(g_reader, len, cmd);
	}
	else if (g_type == "udp") 
	{
		SendUdpCmd(g_reader, len, cmd);
	}
	else if (g_type == "tcp")
	{
#if 0
		if ( send(g_tcp_socket, cmd, 6, 0) != 6)
		{
			ROS_ERROR("Stop motor, send tcp error");
			return false;
		}
		ROS_INFO("Stop motor");
		return true;
#endif
	}
	return false;
}


void PublishData(HPublish pub, int n, RawData** fans)
{
	int skip = 0;
	RawData* drop[MAX_FANS];
	PubHub* hub = (PubHub*)pub;

	pthread_mutex_lock(&hub->mtx);

	for (int i=0; i<n; i++) 
	{
		if (hub->nfan >= MAX_FANS) 
		{
			drop[skip++] = fans[i];
			continue;
		}
		// if (hub->nfan == 0 && fans[i]->angle != 0) {
		// drop[skip++] = fans[i];
		// continue;
		// }
		hub->fans[hub->nfan++] = fans[i];
	}
	pthread_mutex_unlock(&hub->mtx);

	for (int i=0; i<skip; i++)
	{
		delete drop[i];
	}
}

int angle_cmp(const void* p1, const void* p2)
{
	const RawData** pp1 = (const RawData**)p1;
	const RawData** pp2 = (const RawData**)p2;
	return (*pp1)->ros_angle - (*pp2)->ros_angle;
}


void resample(RawData* dat, int NN)
{
	int* index = new int[NN];
	double* errs = new double[NN];

	for (int i=0; i<NN; i++) index[i] = -1;

	for (int i=0; i<dat->N; i++) 
	{
		if (dat->points[i].distance == 0) continue;

		int idx = round(double(i*NN) / dat->N);
		if (idx < NN) 
		{
			double err = fabs(double(dat->span * i) /dat->N - double(dat->span * idx) / NN);
			if (index[idx] == -1 || err < errs[idx]) {
				index[idx] = i;
				errs[idx] = err;
			}
		}
	}

	for (int i=1; i<NN; i++) 
	{
		if (index[i] != -1) 
		{
			dat->points[i] = dat->points[index[i]];
		}
		else {
			dat->points[i].distance = 0;
			dat->points[i].confidence = 0;
		}
	}

	dat->N = NN;

	delete index;
	delete errs;
}


bool GetFan(HPublish pub, bool with_resample, double resample_res, RawData** fans)
{
	bool got = false;
	PubHub* hub = (PubHub*)pub;

	pthread_mutex_lock(&hub->mtx);

	if (hub->nfan > 0)
	{
		fans[0] = hub->fans[0];
		for (int i=1; i<hub->nfan; i++)
			hub->fans[i-1] = hub->fans[i];
		hub->nfan--;
		got = true;
	}

	pthread_mutex_unlock(&hub->mtx);

	if (with_resample)// && resample_res > 0.05)
       	{
	       	int NN = fans[0]->span/(10*resample_res);
	       	if (NN < fans[0]->N) resample(fans[0], NN);
       	}

	return got;
}

int GetAllFans(HPublish pub, bool with_resample, double resample_res, RawData** fans)
{
	PubHub* hub = (PubHub*)pub;

	//RawData* drop[MAX_FANS];
	pthread_mutex_lock(&hub->mtx);

	int cnt = 0;
	for (int i=0; i<hub->nfan; i++)
	{
		if (hub->fans[i]->angle + hub->fans[i]->span == 3600)
		{
			cnt = i+1;
			break;
		}
	}

	if (cnt > 0) 
	{
		for (int i=0; i< cnt; i++) 
		{
			fans[i] = hub->fans[i];
		}
		for (int i=cnt; i<hub->nfan; i++)
			hub->fans[i-cnt] = hub->fans[i];
		hub->nfan -= cnt;
	}

	pthread_mutex_unlock(&hub->mtx);

	if (cnt > 0) 
	{
		bool circle = true;
		int total = fans[0]->span;;
		for (int i=0; i<cnt-1; i++)
		{
			if (fans[i]->angle + fans[i]->span != fans[i+1]->angle)
				circle = false;
		       	total += fans[i+1]->span;
		}

		if (!circle || total != 3600) {
			printf("%d drop %d fans\n", total, cnt);
			// clean imcomplent datas
			for (int i=0; i<cnt; i++) delete fans[i];
			cnt = 0;
		}
	}
	if (cnt > 0)
	{
		qsort(fans, cnt, sizeof(RawData*), angle_cmp);

		if (with_resample)
		{
			for (int i=0; i<cnt; i++) 
			{
				int NN = fans[i]->span/(10*resample_res);
				if (NN < fans[i]->N) {
					resample(fans[i], NN);
				}
			}
		}
	}

	return cnt;
}

short LidarAng2ROS(short ang);

void PublishLaserScan(ros::Publisher& laser_pub, RawData* fan, std::string& frame_id, double max_dist)
{
	sensor_msgs::LaserScan msg;

	msg.header.stamp = ros::Time::now();
       	msg.header.frame_id = frame_id;

	double scan_time = 1/10.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;
		
	msg.angle_min = (LidarAng2ROS(fan->angle + fan->span)) * M_PI/1800; 
	msg.angle_max = msg.angle_min + fan->span * M_PI/1800; 

	msg.angle_increment = (fan->span * M_PI/1800) / fan->N;

	msg.range_min = 0.; 
	msg.range_max = max_dist;//8.0; 

	msg.intensities.resize(fan->N); 
	msg.ranges.resize(fan->N);

	for (int i=fan->N-1; i>=0; i--)
	{
		double d = fan->points[i].distance/1000.0;
		if (fan->points[i].distance == 0 || d > max_dist) 
			msg.ranges[i] = std::numeric_limits<float>::infinity();
		else
			msg.ranges[i] = d;

		msg.intensities[i] = fan->points[i].confidence;
	} 
	laser_pub.publish(msg); 
}


void PublishLaserScan(ros::Publisher& laser_pub, int nfan, RawData** fans, std::string& frame_id, double max_dist)
{
	sensor_msgs::LaserScan msg;

	int N = 0;
	for (int i=0; i<nfan; i++) N += fans[i]->N;

	msg.header.stamp = ros::Time::now();
       	msg.header.frame_id = frame_id;

	double scan_time = 1/10.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / N;
		
	msg.angle_min = -M_PI;
	msg.angle_max = M_PI;
	msg.angle_increment = M_PI*2 / N;

	msg.range_min = 0.; 
	msg.range_max = max_dist;//8.0; 

	msg.intensities.resize(N); 
	msg.ranges.resize(N);

	N = 0;
	for (int j=0; j<nfan; j++) 
	{
		for (int i=fans[j]->N-1; i>=0; i--) 
		{
			double d = fans[j]->points[i].distance/1000.0;
			if (fans[j]->points[i].distance == 0 || d > max_dist) 
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fans[j]->points[i].confidence;
			N++;
		}
	} 
	laser_pub.publish(msg); 
}

void PublishCloud(ros::Publisher& cloud_pub, int nfan, RawData** fans, std::string& frame_id, double max_dist)
{
	sensor_msgs::PointCloud cloud; 
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = frame_id; 

	int N = 0;
	for (int i=0; i<nfan; i++) N += fans[i]->N;

	cloud.points.resize(N);
	cloud.channels.resize(1); 
	cloud.channels[0].name = "intensities"; 
	cloud.channels[0].values.resize(N);

	int idx = 0;
	for (int j=0; j<nfan; j++) 
	{
		for (int i=0; i<fans[j]->N; i++) 
		{
			float r = fans[j]->points[i].distance/1000.0 ; 
			// float a = j*M_PI/5 + i*M_PI/5/dat360[j].N;
			float a = -fans[j]->points[i].degree * M_PI/180;
			cloud.points[idx].x = cos(a) * r;
			cloud.points[idx].y = sin(a) * r;
			cloud.points[idx].z = 0;
			cloud.channels[0].values[idx] = fans[j]->points[i].confidence;
			idx++;
		}
	} 
	cloud_pub.publish(cloud);
}


void setup_params(bluesea2::DynParamsConfig &config, uint32_t level) 
{
	ROS_INFO("Change RPM to [%d]", config.rpm);

	char cmd[32];
	sprintf(cmd, "LSRPM:%dH", config.rpm);

	SendCmd(strlen(cmd), cmd);
}


bool should_start = true;
// service call back function
bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	should_start = false;
	ROS_INFO("Stop motor");
	char cmd[] = "LSTOPH";
	return SendCmd(6, cmd);
}

// service call back function
bool start_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	should_start = true;
	char cmd[] = "LSTARH";

	ROS_INFO("Stop motor");

	return SendCmd(6, cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea2_laser_publisher");
       	ros::NodeHandle n;
       	ros::NodeHandle priv_nh("~");
	
	std_msgs::UInt16 rpms; 

	// LiDAR comm type, could be "uart" or "udp"
	priv_nh.param("type", g_type, std::string("uart")); 
	printf("type is %s\n", g_type.c_str());

#if 0
	// dump raw data for debug
	std::string dump_path;
	priv_nh.param("dump", dump_path, std::string("")); 

	FILE* fp_dump = NULL;
	if (!dump_path.empty())
		fp_dump = fopen(dump_path.c_str(), "wb");
#endif

	//////////////////////////////////////////////////////////////
	// for serial port comm
	std::string port;
	std::vector<int> rate_list;
       	int baud_rate;
	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
       	priv_nh.param("baud_rate", baud_rate, 256000);

	priv_nh.getParam("rate_list", rate_list);

	// for network comm
	std::string lidar_ip, group_ip;
	priv_nh.param("lidar_ip", lidar_ip, std::string("192.168.158.91"));
	priv_nh.param("group_ip", group_ip, std::string("192.168.158.91"));
	int lidar_port, local_port;
	priv_nh.param("lidar_port", lidar_port, 5000);
	priv_nh.param("local_port", local_port, 50122);

	// raw data format
	int raw_bytes, normal_size;
	bool unit_is_mm, with_confidence, with_chk;
	priv_nh.param("raw_bytes", raw_bytes, 3); // packet mode : 2bytes or 3bytes
       	priv_nh.param("normal_size", normal_size, -1); // -1 : allow all packet, N : drop packets whose points less than N 
	priv_nh.param("unit_is_mm", unit_is_mm, true); // 0 : distance is CM, 1: MM
       	priv_nh.param("with_confidence", with_confidence, true); // 
       	priv_nh.param("with_checksum", with_chk, true); // 1 : enable packet checksum


	// angle composate
	bool hard_resample, soft_resample;
       	priv_nh.param("hard_resample", hard_resample, true); // resample angle resolution
       	priv_nh.param("soft_resample", soft_resample, true); // resample angle resolution
	double resample_res;
       	priv_nh.param("resample_res", resample_res, 0.5); // resample angle resolution @ 0.5 degree 
	//int angle_patch;
       	//priv_nh.param("angle_patch", angle_patch, 1); // make points number of every fans to unique
	if (resample_res < 0.05 || resample_res > 1) {
		soft_resample = false;
		hard_resample = false;
	}

	// data output
	bool output_scan, output_cloud, output_360;
	priv_nh.param("output_scan", output_scan, true); // true: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", output_cloud, false); // false: enable output xyz format, 0 : disable
	priv_nh.param("output_360", output_360, true); // true: packet data of 360 degree (multiple RawData), publish once
							// false: publish every RawData (36 degree)
							
	// angle filter
	int with_angle_filter;
	double min_angle, max_angle;
	priv_nh.param("with_angle_filter", with_angle_filter, 0); // 1: enable angle filter, 0: diable
	priv_nh.param("min_angle", min_angle, -M_PI); // angle filter's low threshold, default value: -pi
       	priv_nh.param("max_angle", max_angle, M_PI); // angle filters' up threashold, default value: pi

	// range limitation
	double max_dist;
       	priv_nh.param("max_dist", max_dist, 9999.0); // max detection range, default value: 9999M

	// frame information
       	std::string frame_id;
       	int firmware_number; 
       	priv_nh.param("frame_id", frame_id, std::string("LH_laser")); // could be used for rviz
       	priv_nh.param("firmware_version", firmware_number, 2);

#if 0
	// output data format
	int mirror, from_zero;
       	priv_nh.param("mirror", mirror, 0); // 0: clockwise, 1: counterclockwise
       	priv_nh.param("from_zero", from_zero, 0); // 1: angle range [0 - 360), 0: angle range [-180, 180)
#endif
       	
	ros::Publisher laser_pub, cloud_pub;

	if (output_cloud)
		cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 50);
	if (output_scan)
		laser_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
   
	ros::ServiceServer stop_srv = n.advertiseService("stop_motor", stop_motor);
       	ros::ServiceServer start_srv = n.advertiseService("start_motor", start_motor);

	dynamic_reconfigure::Server<bluesea2::DynParamsConfig> server;
       	server.setCallback( boost::bind(&setup_params, _1, _2) );


	uint32_t flags = 0;
	if (unit_is_mm) flags |= DF_UNIT_IS_MM;
	if (with_confidence) flags |= DF_WITH_INTENSITY;
	if (hard_resample) flags |= DF_WITH_RESAMPLE;

	HParser parser = ParserOpen(raw_bytes, flags, resample_res, with_chk);

	PubHub* hub = new PubHub;
	pthread_mutex_init(&hub->mtx, NULL);


	if (g_type == "uart") 
	{
		int* rates = new int[rate_list.size()+1];
		for (int i=0; i<rate_list.size(); i++)
		{
			rates[i] = rate_list[i];
			printf("[%d] => %d\n", i, rate_list[i]);
		}
		rates[rate_list.size()] = 0;
		g_reader = StartUartReader(port.c_str(), baud_rate, rates, parser, hub);
	}
	else if (g_type == "udp") 
	{
		g_reader = StartUDPReader(lidar_ip.c_str(), lidar_port,
			       	group_ip.c_str(), local_port, parser, hub);
	}


	while (ros::ok()) 
	{ 
		ros::spinOnce();
		
		RawData* fans[MAX_FANS];

		if (!output_360) 
		{
			if (GetFan(hub, soft_resample, resample_res, fans)) 
		       	{
				if (output_scan) 
				       PublishLaserScan(laser_pub, fans[0], frame_id, max_dist);
				//printf("free %x\n", fans[0]);
				delete fans[0];
			}
			else {
				ros::Duration(0.001).sleep();
			}
		}
		else
		{
			int n = GetAllFans(hub, soft_resample, resample_res, fans);
			if (n > 0)
			{ 
				if (output_scan) 
				       PublishLaserScan(laser_pub, n, fans, frame_id, max_dist);
		
				if (output_cloud)
				       PublishCloud(cloud_pub, n, fans, frame_id, max_dist);

				for (int i=0; i<n; i++) delete fans[i];
			}
			else {
			       	ros::Duration(0.001).sleep();
			}
		}
	}

       	return 0;
}




