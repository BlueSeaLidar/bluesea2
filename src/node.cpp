#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <bluesea2/Control.h>
#include <bluesea2/DefenceZone.h>
#include <ros/console.h>
#include <time.h>
#include "../sdk/include/bluesea.h"

BlueSeaLidarDriver *m_driver=NULL;
void PublishLaserScanFan(ros::Publisher &laser_pub, RawData *fan, std::string &frame_id, double min_dist, double max_dist, uint8_t inverted, uint8_t reversed)
{
	sensor_msgs::LaserScan msg;
	msg.header.stamp.sec = fan->ts[0];
	msg.header.stamp.nsec = fan->ts[1];

	msg.header.frame_id = frame_id;
	int N = fan->N;
	double scan_time = 1 / 100.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;
	double min_pos, max_pos;
	msg.intensities.resize(N); // fan->N);
	msg.ranges.resize(N);	   // fan->N);
	if (inverted)
	{
		min_pos = ROSAng(-fan->angle / 10) * 10 * M_PI / 1800;
		max_pos = -min_pos; // + fan->span * M_PI / 1800;
	}
	else
	{
		min_pos = ROSAng(fan->angle / 10) * 10 * M_PI / 1800;
		max_pos = min_pos; //-fan->span  * M_PI / 1800;
	}
	msg.intensities.resize(N); // fan->N);
	msg.ranges.resize(N);	   // fan->N);

	if (inverted)
	{
		msg.angle_min = min_pos;
		msg.angle_max = max_pos;
		msg.angle_increment = (fan->span * M_PI / 1800) / fan->N;
	}
	else
	{
		msg.angle_min = max_pos;
		msg.angle_max = min_pos;
		msg.angle_increment = -(fan->span * M_PI / 1800) / fan->N;
	}
	N = 0;
	if (reversed)
	{
		for (int i = fan->N - 1; i >= 0; i--)
		{
			double deg = ROSAng(fan->points[i].degree);

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist)
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fan->points[i].confidence;
			N++;
		}
	}
	else
	{
		for (int i = 0; i < fan->N; i++)
		{
			double deg = ROSAng(fan->points[i].degree);
			// customize angle filter
			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist)
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fan->points[i].confidence;
			N++;
		}
	}
	laser_pub.publish(msg);
}

void PublishLaserScan(ros::Publisher &laser_pub, HPublish pub, ArgData argdata, uint8_t counterclockwise)
{
	PubHub *hub = (PubHub *)pub;
	sensor_msgs::LaserScan msg;
	int N = hub->consume.size();
	// make  min_ang max_ang  convert to mask
	msg.header.stamp.sec = hub->ts_beg[0];
	msg.header.stamp.nsec = hub->ts_beg[1];

	double ti = double(hub->ts_beg[0]) + double(hub->ts_beg[1]) / 1000000000.0;
	double tx = double(hub->ts_end[0]) + double(hub->ts_end[1]) / 1000000000.0;

	msg.scan_time = tx - ti; // nfan/(nfan-1);
	msg.time_increment = msg.scan_time / N;

	msg.header.frame_id = argdata.frame_id;

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	msg.range_min = argdata.min_dist;
	msg.range_max = argdata.max_dist; // 8.0;
	if (argdata.with_angle_filter)
	{
		double min_pos, max_pos;
		N = GetCount(hub->consume, min_deg, max_deg, min_pos, max_pos);
		if (argdata.inverted)
		{
			msg.angle_min = min_pos * M_PI / 180;
			msg.angle_max = max_pos * M_PI / 180;
			// DEBUG("data1:deg:%lf   %lf  angle:%lf   %lf   %d\n",min_deg,max_deg,msg.angle_min,msg.angle_max,N);
		}
		else
		{
			msg.angle_min = max_pos * M_PI / 180;
			msg.angle_max = min_pos * M_PI / 180;
		}
		msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
	}
	else if (argdata.from_zero)
	{
		if (argdata.inverted)
		{
			msg.angle_min = 0;
			msg.angle_max = 2 * M_PI * (N - 1) / N;
			msg.angle_increment = M_PI * 2 / N;
		}
		else
		{
			msg.angle_min = 2 * M_PI * (N - 1) / N;
			msg.angle_max = 0;
			msg.angle_increment = -M_PI * 2 / N;
		}
	}
	else
	{
		if (argdata.inverted)
		{
			msg.angle_min = -M_PI;
			msg.angle_max = M_PI - (2 * M_PI) / N;
			msg.angle_increment = M_PI * 2 / N;
		}
		else
		{
			msg.angle_min = M_PI;
			msg.angle_max = -M_PI + (2 * M_PI) / N;
			msg.angle_increment = -M_PI * 2 / N;
		}
	}

	if (counterclockwise != 0)
	{
		double d = msg.angle_min;
		msg.angle_min = msg.angle_max;
		msg.angle_max = d;
		msg.angle_increment = -msg.angle_increment;
		msg.angle_min -= msg.angle_increment;
		msg.angle_max -= msg.angle_increment;
	}
	msg.intensities.resize(N);
	msg.ranges.resize(N);
	int idx = 0;

	if (argdata.reversed)
	{
		for (int i = N - 1; i >= 0; i--)
		{
			double deg = ROSAng(hub->consume[i].degree);
			double d = hub->consume[i].distance / 1000.0;
			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = hub->consume[i].confidence;
			idx++;
		}
	}
	else
	{

		// ROS_INFO("%s %d %f %f\n", __FUNCTION__, __LINE__, hub->consume[0].degree,hub->consume[hub->consume.size()-1].degree);
		for (int i = 0; i <= N - 1; i++)
		{
			// DEBUG(" %d %d  %d  %lf \n", datapoint[i].distance,datapoint[i].confidence,N,datapoint[i].degree);
			double deg = ROSAng(hub->consume[i].degree);
			double d = hub->consume[i].distance / 1000.0;
			bool custom = false;
			for (int k = 0; k < argdata.masks.size() && !custom; k++)
			{
				if (argdata.with_angle_filter && argdata.masks[k].min <= deg && deg <= argdata.masks[k].max)
					custom = true;
			}

			if (hub->consume[i].distance == 0 || d > argdata.max_dist || d < argdata.min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = hub->consume[i].confidence;

			idx++;
		}
	}
	hub->consume.clear();
	laser_pub.publish(msg);
}

void PublishCloud(ros::Publisher &cloud_pub, HPublish pub, ArgData argdata)
{
	PubHub *hub = (PubHub *)pub;
	sensor_msgs::PointCloud cloud;
	// cloud.header.stamp = ros::Time::now();

	cloud.header.stamp.sec = hub->ts_beg[0];
	cloud.header.stamp.nsec = hub->ts_beg[1];

	cloud.header.frame_id = argdata.frame_id;

	int N = hub->consume.size();

	cloud.points.resize(N);
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(N);

	double min_deg = argdata.min_angle * 180 / M_PI;
	double max_deg = argdata.max_angle * 180 / M_PI;

	int idx = 0;
	for (int i = 0; i < hub->consume.size(); i++)
	{
		if (argdata.with_angle_filter)
		{
			if (ROSAng(hub->consume[i].degree) < min_deg)
				continue;
			if (ROSAng(hub->consume[i].degree) > max_deg)
				continue;
		}

		float r = hub->consume[i].distance / 1000.0;
		// float a = j*M_PI/5 + i*M_PI/5/dat360[j].N;
		float a = -hub->consume[i].degree * M_PI / 180;

		cloud.points[idx].x = cos(a) * r;
		cloud.points[idx].y = sin(a) * r;
		cloud.points[idx].z = 0;
		cloud.channels[0].values[idx] = hub->consume[i].confidence;
		idx++;
	}
	cloud_pub.publish(cloud);
}

// service call back function
bool stop_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	ROS_INFO("Stop motor  index:%ld", req.index);
	char cmd[] = "LSTOPH";
	return m_driver->sendCmd(6, cmd, req.index);
}

// service call back function
bool start_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	ROS_INFO("Start motor  index:%ld", req.index);
	char cmd[] = "LSTARH";
	return m_driver->sendCmd(6, cmd, req.index);
}
bool switchZone_motor(bluesea2::DefenceZone::Request &req, bluesea2::DefenceZone::Response &res)
{

	char cmd[12] = {0};
	sprintf(cmd, "LSAZN:%xH", req.index);
	ROS_INFO("DefenceZone motor  cmd:%s,ip:%s", cmd, req.ip.c_str());
	int index = req.index;
	if (index < 0)
	{
		return false;
	}
	else
	{
		return m_driver->sendUdpCmd2((char *)req.ip.c_str(), 12, cmd);
	}
	return false;
}

bool rpm_motor(bluesea2::DefenceZone::Request &req, bluesea2::DefenceZone::Response &res)
{

	char cmd[12] = {0};
	sprintf(cmd, "LSRPM:%sH", req.ip.c_str());
	ROS_INFO("RPM motor  cmd:%s\n:",cmd);
	int index = req.index;
	return m_driver->sendCmd(strlen(cmd), cmd, req.index);
}


bool get_range_param(ros::NodeHandle nh, const char *name, Range &range)
{
	std::vector<double> rg;
	if (nh.getParam(name, rg))
	{
		if (rg.size() == 2 && rg[0] < rg[1])
		{
			range.min = rg[0] * 180 / M_PI;
			range.max = rg[1] * 180 / M_PI;
			return true;
		}
	}
	return false;
}

bool ProfileInit(ros::NodeHandle priv_nh, ArgData &argdata)
{
	priv_nh.param("number", argdata.num, 1);
	priv_nh.param("type", argdata.type, std::string("uart"));
	priv_nh.param("frame_id", argdata.frame_id, std::string("LH_laser"));
	priv_nh.param("dev_id", argdata.dev_id, ANYONE); //
	// dual lidar arg
	for (int i = 0; i < argdata.num; i++)
	{
		ConnectArg arg;
		char s[32], t[32];
		if (i == 0)
		{
			if (argdata.type == "udp")
			{
				priv_nh.param("lidar_ip", arg.arg1, std::string("192.168.158.98"));
				priv_nh.param("lidar_port", arg.arg2, 6543);
			}
			else if (argdata.type == "vpc")
			{
				priv_nh.param("port", arg.arg1, std::string("/dev/ttyACM0"));
				priv_nh.param("baud_rate", arg.arg2, 115200);
			}
			else if (argdata.type == "uart")
			{
				priv_nh.param("port", arg.arg1, std::string("/dev/ttyUSB0"));
				priv_nh.param("baud_rate", arg.arg2, 500000);
			}
			else
			{
				ROS_ERROR("Profiles type=%s is not exist!", argdata.type.c_str());
				return false;
			}

			priv_nh.param("topic", arg.laser_topics, std::string("scan"));
			priv_nh.param("cloud_topic", arg.cloud_topics, std::string("cloud"));
		}
		else
		{
			if (argdata.type == "udp")
			{
				sprintf(s, "lidar%d_ip", i);
				priv_nh.param(s, arg.arg1, std::string(""));
				sprintf(s, "lidar%d_port", i);
				priv_nh.param(s, arg.arg2, 0);

				if (arg.arg1.empty() || arg.arg2 == 0)
				{
					ROS_ERROR("Profiles lidar num %d is not exist!", i);
					break;
				}
				sprintf(s, "topic%d", i);
				sprintf(t, "scan%d", i);
				priv_nh.param(s, arg.laser_topics, std::string(t));
				sprintf(s, "cloud_topic%d", i);
				sprintf(t, "cloud%d", i);
				priv_nh.param(s, arg.cloud_topics, std::string(t));
			}
		}
		argdata.connectargs.push_back(arg);
	}
	priv_nh.param("local_port", argdata.localport, 6668);
	// custom
	priv_nh.param("group_listener", argdata.custom.is_group_listener, false);
	priv_nh.param("group_ip", argdata.custom.group_ip, std::string("224.1.1.91"));
	priv_nh.param("raw_bytes", argdata.raw_bytes, 3); // packet mode : 2bytes or 3bytes
													  // is lidar inverted
	bool inverted, reversed;
	// priv_nh.param("inverted", argdata.inverted, false);
	priv_nh.param("reversed", argdata.reversed, false);
	// data output
	priv_nh.param("output_scan", argdata.output_scan, true);	// true: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", argdata.output_cloud, false); // false: enable output xyz format, 0 : disable
	priv_nh.param("output_360", argdata.output_360, true);		// true: packet data of 360 degree (multiple RawData), publish once
																// false: publish every RawData (36 degree)
	//  angle filter
	bool with_angle_filter;
	double min_angle, max_angle;
	priv_nh.param("with_angle_filter", argdata.with_angle_filter, false); // true: enable angle filter, false: disable
	priv_nh.param("min_angle", argdata.min_angle, -M_PI);				  // angle filter's low threshold, default value: -pi
	priv_nh.param("max_angle", argdata.max_angle, M_PI);				  // angle filters' up threashold, default value: pi

	// range limitation
	double min_dist, max_dist;
	priv_nh.param("min_dist", argdata.min_dist, 0.1);  // min detection range, default value: 0M
	priv_nh.param("max_dist", argdata.max_dist, 50.0); // max detection range, default value: 9999M
	// customize angle filter
	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(priv_nh, name, range))
			break;
		argdata.masks.push_back(range);
	}
	priv_nh.param("from_zero", argdata.from_zero, false);
	priv_nh.param("error_circle", argdata.custom.error_circle, 3);
	priv_nh.param("error_scale", argdata.custom.error_scale, 0.9);
	priv_nh.param("time_mode", argdata.time_mode, 0);

	/*******************************FITTER arg start******************************/
	priv_nh.param("filter_open", argdata.fitter.isopen, false);
	priv_nh.param("filter_type", argdata.fitter.type, 1);
	priv_nh.param("max_range", argdata.fitter.max_range, 20.0);
	priv_nh.param("min_range", argdata.fitter.min_range, 0.5);
	priv_nh.param("max_range_difference", argdata.fitter.max_range_difference, 0.1);
	priv_nh.param("filter_window", argdata.fitter.filter_window, 1);
	/*******************************FITTER arg end******************************/

	/*****************************GET arg start************************************/
	priv_nh.param("uuid", argdata.uuid, -1);
	// priv_nh.param("model", argdata.model, -1);
	/*****************************GET arg end************************************/
	/*****************************SET arg start************************************/
	priv_nh.param("rpm", argdata.rpm, -1); // set motor RPM
	// angle composate
	priv_nh.param("hard_resample", argdata.hard_resample, false); // resample angle resolution
	priv_nh.param("soft_resample", argdata.soft_resample, false); // resample angle resolution
	priv_nh.param("resample_res", argdata.resample, -1.0);		  // resample angle resolution

	if (!argdata.hard_resample)
		argdata.resample = -1;

	priv_nh.param("with_smooth", argdata.with_smooth, -1);	   // lidar data smooth filter
	priv_nh.param("with_deshadow", argdata.with_deshadow, -1); // data shadow filter
	priv_nh.param("alarm_msg", argdata.alarm_msg, -1);		   // let lidar upload alarm message
	priv_nh.param("direction", argdata.direction, -1);
	priv_nh.param("unit_is_mm", argdata.unit_is_mm, -1); // 0 : distance is CM, 1: MM
	priv_nh.param("with_confidence", argdata.with_confidence, -1);
	priv_nh.param("ats", argdata.ats, -1);
	/*****************************SET arg end************************************/

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea2_laser_publisher");
	ros::NodeHandle node_handle;
	ros::NodeHandle priv_nh("~");

	ROS_INFO("ROS VERSION:%s\n", BLUESEA2_VERSION);
	//init launch arg   
	ArgData argdata;
	ProfileInit(priv_nh, argdata);
	
	m_driver = new BlueSeaLidarDriver();
	m_driver->getInitCmds(argdata);
	m_driver->openLidarThread();
	//create topic
	ros::Publisher laser_pubs[MAX_LIDARS], cloud_pubs[MAX_LIDARS];
	for (int i = 0; i < argdata.num; i++)
	{
		char s[32];
		if (argdata.output_cloud)
		{
			cloud_pubs[i] = node_handle.advertise<sensor_msgs::PointCloud>(argdata.connectargs[i].cloud_topics, 50);
		}
		if (argdata.output_scan)
		{
			laser_pubs[i] = node_handle.advertise<sensor_msgs::LaserScan>(argdata.connectargs[i].laser_topics, 50);
		}
	}
	//interacting with the client
	ros::ServiceServer stop_srv = node_handle.advertiseService("stop_motor", stop_motor);
	ros::ServiceServer start_srv = node_handle.advertiseService("start_motor", start_motor);
	ros::ServiceServer switchZone_srv = node_handle.advertiseService("switchZone_motor", switchZone_motor);
	ros::ServiceServer rpm_srv = node_handle.advertiseService("rpm_motor", rpm_motor);
	// // get offsetangle
	while (ros::ok())
	{
		ros::spinOnce();
		bool idle = true;
		for (int i = 0; i < argdata.num; i++)
		{
			PubHub *hub = m_driver->getHub(i);
			if (hub->nfan == 0)
				continue;
			argdata.inverted=hub->inverted;
			bool ret = false;
			if (hub->offsetangle == -1)
			{
				ret = m_driver->checkIsRun(i);
				continue;
			}
			if (!argdata.output_360)
			{
				RawData *fans[MAX_FANS] = {NULL};
				if (m_driver->GetFan(hub, argdata.soft_resample, argdata.resample, fans))
				{
					if (argdata.output_scan)
					{
						PublishLaserScanFan(laser_pubs[i], fans[0], argdata.frame_id,
											argdata.min_dist, argdata.max_dist, argdata.inverted, argdata.reversed);
					}
					delete fans[0];
					idle = false;
				}
			}
			else
			{
				// 按照点数判定一帧的数据，然后   零数判定， 离异点过滤，最大最小值过滤(空点去除)， 指定角度屏蔽(mask置零)
				uint8_t counterclockwise = false;
				int n = m_driver->GetAllFans(hub, argdata, counterclockwise);
				if (n > 0)
				{
					idle = false;
					if (argdata.output_scan)
					{
						PublishLaserScan(laser_pubs[i], hub, argdata, counterclockwise);
					}

					if (argdata.output_cloud)
					{
						PublishCloud(cloud_pubs[i], hub, argdata);
					}
				}
			}
		}
		if (idle)
		{
			ros::Duration(0.001).sleep();
		}
	}
	return 0;
}
