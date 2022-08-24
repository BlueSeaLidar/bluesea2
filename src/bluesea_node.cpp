
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
#include <bluesea2/Control.h>
#include <dynamic_reconfigure/server.h>
#include <bluesea2/DynParamsConfig.h>
#include <time.h>
#include <sys/stat.h>
#include <pthread.h>
#include <signal.h>
#include "reader.h"

HReader g_reader = NULL;
std::string g_type = "uart";

struct Range
{
	double min;
	double max;
};

struct PubHub
{
	pthread_mutex_t mtx;
	int nfan;
	RawData *fans[MAX_FANS];
};

void closeSignal(int sig)
{
	//这里主�?�进行退出前的数�?保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
	exit(0);
}

bool SendCmd(int len, char *cmd, int index)
{
	if (g_type == "uart")
	{
		return SendUartCmd(g_reader, len, cmd);
	}
	else if (g_type == "udp")
	{
		if (index < 0)
		{
			int tmp = 0;
			memcpy(&tmp, g_reader, sizeof(int));
			for (int i = 0; i < tmp; i++)
			{
				SendUdpCmd(g_reader, i, len, cmd);
			}
			return true;
		}
		else
			return SendUdpCmd(g_reader, index, len, cmd);
	}
	else if (g_type == "tcp")
	{
		return SendTcpCmd(g_reader, len, cmd);
	}
	return false;
}

void PublishData(HPublish pub, int n, RawData **fans)
{
	int skip = 0;
	RawData *drop[MAX_FANS];
	PubHub *hub = (PubHub *)pub;

	pthread_mutex_lock(&hub->mtx);

	if (hub->nfan + n > MAX_FANS) 
	{
		int nr = hub->nfan + n - MAX_FANS;

		for (int i=0; i<nr; i++)
			drop[skip++] = hub->fans[i];

		for (int i=nr; i<hub->nfan; i++)
		{
			hub->fans[i-nr] = hub->fans[i];
		}

		hub->nfan -= nr;
	}

	for (int i=0; i<n; i++) 
	{
		hub->fans[hub->nfan++] = fans[i];
	}

	pthread_mutex_unlock(&hub->mtx);

	for (int i = 0; i < skip; i++)
	{
		delete drop[i];
	}
}

#if 0
int angle_cmp(const void* p1, const void* p2)
{
	const RawData** pp1 = (const RawData**)p1;
	const RawData** pp2 = (const RawData**)p2;
	return (*pp1)->ros_angle - (*pp2)->ros_angle;
}
#endif

void resample(RawData *dat, int NN)
{
	int *index = new int[NN];
	double *errs = new double[NN];

	for (int i = 0; i < NN; i++)
		index[i] = -1;

	for (int i = 0; i < dat->N; i++)
	{
		if (dat->points[i].distance == 0)
			continue;

		int idx = round(double(i * NN) / dat->N);
		if (idx < NN)
		{
			double err = fabs(double(dat->span * i) / dat->N - double(dat->span * idx) / NN);
			if (index[idx] == -1 || err < errs[idx])
			{
				index[idx] = i;
				errs[idx] = err;
			}
		}
	}

	for (int i = 1; i < NN; i++)
	{
		if (index[i] != -1)
		{
			dat->points[i] = dat->points[index[i]];
		}
		else
		{
			dat->points[i].distance = 0;
			dat->points[i].confidence = 0;
		}
		dat->points[i].degree = dat->angle / 10.0 + (dat->span * i) / (10.0 * NN);
	}

	dat->N = NN;

	delete index;
	delete errs;
}

bool GetFan(HPublish pub, bool with_resample, double resample_res, RawData **fans)
{
	bool got = false;
	PubHub *hub = (PubHub *)pub;

	pthread_mutex_lock(&hub->mtx);

	if (hub->nfan > 0)
	{
		fans[0] = hub->fans[0];
		for (int i = 1; i < hub->nfan; i++)
			hub->fans[i - 1] = hub->fans[i];
		hub->nfan--;
		got = true;
	}

	pthread_mutex_unlock(&hub->mtx);

	if (with_resample) // && resample_res > 0.05)
	{
		int NN = fans[0]->span / (10 * resample_res);
		if (NN < fans[0]->N)
		{
			resample(fans[0], NN);
		}
		else if (NN > fans[0]->N)
		{
			printf("fan %d less than %d\n", fans[0]->N, NN);
		}
	}

	return got;
}

int GetAllFans(HPublish pub, bool with_resample, double resample_res, RawData **fans, bool from_zero,
			   uint32_t *ts_beg, uint32_t *ts_end)
{
	PubHub *hub = (PubHub *)pub;
	// RawData* drop[MAX_FANS];
	pthread_mutex_lock(&hub->mtx);
	int cnt = 0;
	for (int i = 1; i < hub->nfan; i++)
	{
		if ((from_zero && hub->fans[i]->angle == 0) ||
			(!from_zero && hub->fans[i]->angle == 1800))
		{
			ts_end[0] = hub->fans[i]->ts[0];
			ts_end[1] = hub->fans[i]->ts[1];
			cnt = i;
			break;
		}
	}

	if (cnt > 0)
	{
		for (int i = 0; i < cnt; i++)
		{
			fans[i] = hub->fans[i];
		}
		for (int i = cnt; i < hub->nfan; i++)
			hub->fans[i - cnt] = hub->fans[i];
		hub->nfan -= cnt;
	}
	pthread_mutex_unlock(&hub->mtx);

	if (cnt > 0)
	{
		bool circle = true;
		int total = fans[0]->span;
		ts_beg[0] = fans[0]->ts[0];
		ts_beg[1] = fans[0]->ts[1];
		// printf("ts = %d.%d\n",fans[0]->ts[0],fans[0]->ts[1]);
		for (int i = 0; i < cnt - 1; i++)
		{
			if (fans[i]->angle + fans[i]->span != fans[i + 1]->angle &&
				fans[i]->angle + fans[i]->span != 3600)
				circle = false;
			total += fans[i + 1]->span;
		}
		if (!circle || total != 3600)
		{
			printf("%d drop %d fans\n", total, cnt);
			// clean imcomplent datas
			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
	}
	if (cnt > 0)
	{
		if (with_resample)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * resample_res);
				if (NN < fans[i]->N)
				{
					resample(fans[i], NN);
				}
				else if (NN > fans[i]->N)
				{
					printf("fan [%d] %d less than %d\n", i, fans[i]->N, NN);
				}
			}
		}
	}

	return cnt;
}

void SetTimeStamp(RawData *dat)
{
	ros::Time t = ros::Time::now();
	dat->ts[0] = t.sec;
	dat->ts[1] = t.nsec;
}

double ROSAng(double ang)
{
	ang = -ang;
	return ang < -180 ? ang + 360 : ang;
	// return ang < 180 ? ang : ang - 360;
}

int GetCount(int nfan, RawData **fans, double min_deg, double max_deg, double &min_pos, double &max_pos)
{
	int N = 0, cnt = 0;

	for (int j = 0; j < nfan; j++)
	{
		for (int i = fans[j]->N - 1; i >= 0; i--, cnt++)
		{
			double deg = ROSAng(fans[j]->points[i].degree);
			if (deg < min_deg || deg > max_deg)
				continue;
			if (N == 0)
			{
				min_pos = deg;
				max_pos = deg;
			}
			else
			{
				if (min_pos > deg)
					min_pos = deg;
				if (max_pos < deg)
					max_pos = deg;
			}
			N++;
		}
	}
	// printf("angle filter [%f, %f] %d to %d, [%f, %f]\n", min_deg, max_deg, cnt, N, min_pos, max_pos);
	return N;
}

void PublishLaserScanFan(ros::Publisher &laser_pub, RawData *fan,
						 std::string &frame_id,
						 double min_dist, double max_dist,
						 bool with_filter, double min_ang, double max_ang,
						 bool inverted, bool reversed, double zero_shift,
						 const std::vector<Range> &custom_masks)
{
	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;
	int N = fan->N;
	
	if(zero_shift>M_PI||zero_shift<-M_PI)
	{
		int num = zero_shift/M_PI;
		zero_shift=zero_shift-num*M_PI;
	}	
	if(zero_shift!=0)
	{
		double zero_shift_tmp=zero_shift* 180 / M_PI;
		for (int i = 0; i <N; i++)
		{
			double deg =fan->points[i].degree+zero_shift_tmp;
			fan->points[i].degree=deg;
		}
	
	}
	
	double min_pos, max_pos;
	if (with_filter)
	{
		N = GetCount(1, &fan, min_deg, max_deg, min_pos, max_pos);
		if (N < 2)
			return;
		min_pos = min_pos * M_PI / 180;
		max_pos = max_pos * M_PI / 180;
	}
	else
	{
		if (inverted)
		{
			min_pos = ROSAng(fan->angle / 10) * 10 * M_PI / 1800;
			max_pos = min_pos + fan->span * M_PI / 1800;
		}
		else
		{

			min_pos = ROSAng(fan->angle / 10) * 10 * M_PI / 1800;
			max_pos = min_pos - fan->span * M_PI / 1800;
		}
	}

	sensor_msgs::LaserScan msg;

	// msg.header.stamp = ros::Time::now();
	msg.header.stamp.sec = fan->ts[0];
	msg.header.stamp.nsec = fan->ts[1];

	msg.header.frame_id = frame_id;

	double scan_time = 1 / 100.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;

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

	//msg.angle_min += zero_shift;
	//msg.angle_max += zero_shift;

	N = 0;
	if (reversed)
	{
		for (int i = fan->N - 1; i >= 0; i--)
		{
			double deg = ROSAng(fan->points[i].degree);
			if (with_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			// customize angle filter
			bool custom = false;
			for (int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
					custom = true;
			}

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
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
			if (with_filter)
			{
				if (deg < min_deg)
					continue;
				if (deg > max_deg)
					continue;
			}

			// customize angle filter
			bool custom = false;
			for (int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
					custom = true;
			}

			double d = fan->points[i].distance / 1000.0;

			if (fan->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
				msg.ranges[N] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[N] = d;

			msg.intensities[N] = fan->points[i].confidence;
			N++;
		}
	}
	laser_pub.publish(msg);
}

int time_cmp(const uint32_t *t1, const uint32_t *t2)
{
	if (t1[0] > t2[0])
		return 1;
	else if (t1[0] < t2[0])
		return -1;
	else if (t1[1] > t2[1])
		return 1;
	else if (t1[1] < t2[1])
		return -1;

	return 0;
}

uint32_t last_ns = 0;
void PublishLaserScan(ros::Publisher &laser_pub, int nfan, RawData **fans, std::string &frame_id,
					  double min_dist, double max_dist, bool with_filter, double min_ang, double max_ang,
					  bool inverted, bool reversed, double zero_shift,
					  bool from_zero, uint32_t *ts_beg, uint32_t *ts_end,
					  const std::vector<Range> &custom_masks)
{
	sensor_msgs::LaserScan msg;
	int N = 0;
	if(zero_shift>M_PI||zero_shift<-M_PI)
	{
		int num = zero_shift/M_PI;
		zero_shift=zero_shift-num*M_PI;
	}
		
	double zero_shift_tmp=zero_shift* 180 / M_PI;
	for (int j = 0; j < nfan; j++)
	{
		N += fans[j]->N;
		if(zero_shift==0)
			continue;
		for (int i = 0; i < fans[j]->N; i++)
		{
			double deg =fans[j]->points[i].degree+zero_shift_tmp;
			fans[j]->points[i].degree=deg;
		}
	}

	msg.header.stamp.sec = ts_beg[0];
	msg.header.stamp.nsec = ts_beg[1];

	double ti = double(ts_beg[0]) + double(ts_beg[1]) / 1000000000.0;
	double tx = double(ts_end[0]) + double(ts_end[1]) / 1000000000.0;

	msg.scan_time = tx - ti; // nfan/(nfan-1);
	msg.time_increment = msg.scan_time / N;

	msg.header.frame_id = frame_id;

	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;

	//min_deg -= zero_shift* 180 / M_PI;
	//max_deg -= zero_shift* 180 / M_PI;
	if (with_filter)
	{
		double min_pos, max_pos;
		N = GetCount(nfan, fans, min_deg, max_deg, min_pos, max_pos);
		if (inverted)
		{
			msg.angle_min = min_pos * M_PI / 180;
			msg.angle_max = max_pos * M_PI / 180;
			//printf("data1:deg:%lf   %lf  angle:%lf   %lf   %d\n",min_deg,max_deg,msg.angle_min,msg.angle_max,N);
		}
		else
		{
			msg.angle_min = max_pos * M_PI / 180;
			msg.angle_max = min_pos * M_PI / 180;
		}
		msg.angle_increment = (msg.angle_max - msg.angle_min) / (N - 1);
	}
	else
	{
		if (from_zero)
		{
			if (inverted)
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
			if (inverted)
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
	}

	//msg.angle_min += zero_shift;
	//msg.angle_max += zero_shift;
	//printf("angle:%lf   %lf   %d\n",msg.angle_min,msg.angle_max,N);

	if (fans[0]->counterclockwise != 0)
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
	
	N = 0;
	if (reversed)
	{
		for (int j = nfan - 1; j >= 0; j--)
		{
			for (int i = fans[j]->N - 1; i >= 0; i--)
			{
				double deg = ROSAng(fans[j]->points[i].degree);
				if (with_filter)
				{
					if (deg < min_deg)
						continue;
					if (deg > max_deg)
						continue;
				}

				double d = fans[j]->points[i].distance / 1000.0;

				// customize angle filter
				bool custom = false;
				for (int k = 0; k < custom_masks.size() && !custom; k++)
				{
					if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
						custom = true;
				}

				if (fans[j]->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
					msg.ranges[N] = std::numeric_limits<float>::infinity();
				else
					msg.ranges[N] = d;

				msg.intensities[N] = fans[j]->points[i].confidence;
				N++;
			}
		}
	}
	else
	{
		for (int j = 0; j < nfan; j++)
		{
			for (int i = 0; i < fans[j]->N; i++)
			{
				double deg = ROSAng(fans[j]->points[i].degree);
				
				if (with_filter)
				{
					if (deg < min_deg)
						continue;
					if (deg > max_deg)
						continue;
				}
				// customize angle filter
				bool custom = false;
				for (int k = 0; k < custom_masks.size() && !custom; k++)
				{
					if (with_filter && custom_masks[k].min < deg && deg < custom_masks[k].max)
						custom = true;
				}

				double d = fans[j]->points[i].distance / 1000.0;
				if (fans[j]->points[i].distance == 0 || d > max_dist || d < min_dist || custom)
					msg.ranges[N] = std::numeric_limits<float>::infinity();
				else
					msg.ranges[N] = d;

				
				msg.intensities[N] = fans[j]->points[i].confidence;
				N++;
			}
		}
	}
	
	laser_pub.publish(msg);
}

void PublishCloud(ros::Publisher &cloud_pub, int nfan, RawData **fans, std::string &frame_id,
				  double max_dist,
				  bool with_filter, double min_ang, double max_ang)
{
	sensor_msgs::PointCloud cloud;
	// cloud.header.stamp = ros::Time::now();

	cloud.header.stamp.sec = fans[0]->ts[0];
	cloud.header.stamp.nsec = fans[0]->ts[1];

	cloud.header.frame_id = frame_id;

	int N = 0;
	for (int i = 0; i < nfan; i++)
		N += fans[i]->N;

	cloud.points.resize(N);
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(N);

	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;
	if (with_filter)
	{
		double min_pos, max_pos;
		N = GetCount(nfan, fans, min_deg, max_deg, min_pos, max_pos);
	}

	int idx = 0;
	for (int j = 0; j < nfan; j++)
	{
		for (int i = 0; i < fans[j]->N; i++)
		{
			if (with_filter)
			{
				if (ROSAng(fans[j]->points[i].degree) < min_deg)
					continue;
				if (ROSAng(fans[j]->points[i].degree) > max_deg)
					continue;
			}

			float r = fans[j]->points[i].distance / 1000.0;
			// float a = j*M_PI/5 + i*M_PI/5/dat360[j].N;
			float a = -fans[j]->points[i].degree * M_PI / 180;

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

	SendCmd(strlen(cmd), cmd, 0);
}

bool should_start = true;
// service call back function
bool stop_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	should_start = false;
	ROS_INFO("Stop motor  index:%ld", req.index);
	char cmd[] = "LSTOPH";
	return SendCmd(6, cmd, req.index);
}

// service call back function
bool start_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	should_start = true;
	char cmd[] = "LSTARH";
	ROS_INFO("Start motor  index:%ld", req.index);
	return SendCmd(6, cmd, req.index);
}

uint32_t get_device_ability(const std::string &platform)
{
	if (platform == "LDS-50C-E")
	{
		return // DF_UNIT_IS_MM | DF_WITH_INTENSITY |
			DF_DESHADOWED |
			DF_SMOOTHED |
			DF_WITH_RESAMPLE |
			DF_WITH_UUID |
			EF_ENABLE_ALARM_MSG;
	}
	else if (platform == "LDS-50C-2")
	{
		return DF_UNIT_IS_MM |
			   DF_WITH_INTENSITY |
			   DF_DESHADOWED |
			   DF_SMOOTHED |
			   DF_WITH_UUID;
	}
	else if (platform == "LDS-50C-S")
	{
		return DF_UNIT_IS_MM | DF_WITH_INTENSITY;
	}

	// printf("set with uuid\n");
	return DF_WITH_UUID;
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea2_laser_publisher");
	ros::NodeHandle node_handle;
	ros::NodeHandle priv_nh("~");

	// std_msgs::UInt16 rpms;

	// LiDAR comm type, could be "uart" or "udp"
	priv_nh.param("type", g_type, std::string("uart"));
	printf("type is %s\n", g_type.c_str());

	std::string platform;
	priv_nh.param("platform", platform, std::string("LDS-50C-2"));

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
	int baud_rate;
	priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
	priv_nh.param("baud_rate", baud_rate, 500000);

	std::vector<int> rate_list;
	priv_nh.getParam("rate_list", rate_list);

	// for network comm

	int lidar_ports[MAX_LIDARS], local_port;
	priv_nh.param("lidar_port", lidar_ports[0], 5000);
	priv_nh.param("local_port", local_port, 50122);

	std::string lidar_ips[MAX_LIDARS];
	priv_nh.param("lidar_ip", lidar_ips[0], std::string("192.168.158.91"));
	
	//printf("123:%s lidars\n", lidar_ips[0].c_str());
	
	std::string laser_topics[MAX_LIDARS];
	std::string cloud_topics[MAX_LIDARS];
	priv_nh.param("topic", laser_topics[0], std::string("scan"));
	priv_nh.param("cloud_topic", cloud_topics[0], std::string("cloud"));

	int lidar_count = 1;

	for (int i = 1; i < MAX_LIDARS; i++)
	{
		char s[32], t[32];

		sprintf(s, "lidar%d_ip", i);
		if (lidar_ips[i].length() == 0)
			break;

		sprintf(s, "lidar%d_port", i);
		priv_nh.param(s, lidar_ports[i], 0);
		if (lidar_ports[i] <= 0)
			break;

		sprintf(s, "topic%d", i);
		sprintf(t, "scan%d", i);
		priv_nh.param(s, laser_topics[i], std::string(t));

		sprintf(s, "cloud_topic%d", i);
		sprintf(t, "cloud%d", i);
		priv_nh.param(s, cloud_topics[i], std::string(t));

		lidar_count++;
	}
	printf("we will connect to %d lidars\n", lidar_count);
	for (int i = 0; i < lidar_count; i++)
	{
		printf("lidar[%d] address %s:%d, topic `%s\n", i,
			   lidar_ips[i].c_str(), lidar_ports[i],
			   laser_topics[i].c_str());
	}

	bool is_group_listener;
	priv_nh.param("group_listener", is_group_listener, false);
	std::string group_ip;
	priv_nh.param("group_ip", group_ip, std::string("224.1.1.91"));

	// device identity in data packets, used when multiple lidars connect to single controller
	int dev_id;
	priv_nh.param("dev_id", dev_id, ANYONE); //

	// raw data format
	int raw_bytes, normal_size;
	bool unit_is_mm, with_confidence, with_chk;
	priv_nh.param("raw_bytes", raw_bytes, 3);				 // packet mode : 2bytes or 3bytes
	priv_nh.param("normal_size", normal_size, -1);			 // -1 : allow all packet, N : drop packets whose points less than N
	priv_nh.param("unit_is_mm", unit_is_mm, true);			 // 0 : distance is CM, 1: MM
	priv_nh.param("with_confidence", with_confidence, true); //
	priv_nh.param("with_checksum", with_chk, true);			 // true : enable packet checksum

	// is lidar inverted
	bool inverted, reversed;
	priv_nh.param("inverted", inverted, false);
	priv_nh.param("reversed", reversed, false);

	// zero position
	double zero_shift;
	priv_nh.param("zero_shift", zero_shift, 0.0);

	bool enable_alarm_msg;
	priv_nh.param("alarm_msg", enable_alarm_msg, false); // let lidar upload alarm message

	bool with_smooth, with_deshadow;
	priv_nh.param("with_smooth", with_smooth, true);	 // lidar data smooth filter
	priv_nh.param("with_deshadow", with_deshadow, true); // data shadow filter

	// angle composate
	bool hard_resample, with_soft_resample;
	priv_nh.param("hard_resample", hard_resample, true);	  // resample angle resolution
	priv_nh.param("soft_resample", with_soft_resample, true); // resample angle resolution
	double resample_res;
	priv_nh.param("resample_res", resample_res, 0.5); // resample angle resolution @ 0.5 degree
	// int angle_patch;
	// priv_nh.param("angle_patch", angle_patch, 1); // make points number of every fans to unique
	if (resample_res < 0.05 || resample_res > 1)
	{
		with_soft_resample = false;
		hard_resample = false;
	}

	// data output
	bool output_scan = true, output_cloud = false, output_360 = true;
	priv_nh.param("output_scan", output_scan, true);	// true: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", output_cloud, false); // false: enable output xyz format, 0 : disable
	priv_nh.param("output_360", output_360, true);		// true: packet data of 360 degree (multiple RawData), publish once
														// false: publish every RawData (36 degree)

	// RPM
	int init_rpm;
	priv_nh.param("rpm", init_rpm, -1); // set motor RPM

	// angle filter
	bool with_angle_filter;
	double min_angle, max_angle;
	priv_nh.param("with_angle_filter", with_angle_filter, false); // true: enable angle filter, false: disable
	priv_nh.param("min_angle", min_angle, -M_PI);				  // angle filter's low threshold, default value: -pi
	priv_nh.param("max_angle", max_angle, M_PI);				  // angle filters' up threashold, default value: pi

	// range limitation
	double min_dist, max_dist;
	priv_nh.param("min_dist", min_dist, 0.0);	 // min detection range, default value: 0M
	priv_nh.param("max_dist", max_dist, 9999.0); // max detection range, default value: 9999M

	// customize angle filter
	std::vector<Range> custom_masks;
	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(priv_nh, name, range))
			break;
		custom_masks.push_back(range);
	}

	// frame information
	std::string frame_id;
	priv_nh.param("frame_id", frame_id, std::string("LH_laser")); // could be used for rviz

	int firmware_number;
	priv_nh.param("firmware_version", firmware_number, 2);

	// output data format
	// bool mirror;
	// priv_nh.param("mirror", mirror, 0); // 0: clockwise, 1: counterclockwise
	bool from_zero = false;
	priv_nh.param("from_zero", from_zero, false); // true : angle range [0 - 360), false: angle range [-180, 180)

	bool Savelog = false;
	std::string logPathTmp;
	priv_nh.param("Savelog", Savelog, false);
	//Synthesize the full  log path
	priv_nh.param("logPath", logPathTmp, std::string("/opt/log"));
	char logPath[256] = {0};
	struct stat buffer;
	if (stat(logPathTmp.c_str(), &buffer) != 0)
	{
		mkpathAll(logPathTmp, 0755);
	}
	time_t now;
	struct tm *tm_now;
	time(&now);
	tm_now = localtime(&now);

	sprintf(logPath, "%slog_%04d%02d%02d_%02d%02d%02d.txt", logPathTmp.c_str(), tm_now->tm_year+1900, tm_now->tm_mon, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);

	ros::Publisher laser_pubs[MAX_LIDARS], cloud_pubs[MAX_LIDARS];

	for (int i = 0; i < lidar_count; i++)
	{
		char s[32];
		if (output_cloud)
		{
			cloud_pubs[i] = node_handle.advertise<sensor_msgs::PointCloud>(cloud_topics[i], 50);
		}
		if (output_scan)
		{
			laser_pubs[i] = node_handle.advertise<sensor_msgs::LaserScan>(laser_topics[i], 50);
		}
	}
	ros::ServiceServer stop_srv = node_handle.advertiseService("stop_motor", stop_motor);
	ros::ServiceServer start_srv = node_handle.advertiseService("start_motor", start_motor);

	dynamic_reconfigure::Server<bluesea2::DynParamsConfig> server;
	server.setCallback(boost::bind(&setup_params, _1, _2));

	//
	uint32_t device_ability = get_device_ability(platform);

	//
	uint32_t init_states = 0;
	if (unit_is_mm)
		init_states |= DF_UNIT_IS_MM;
	if (with_confidence)
		init_states |= DF_WITH_INTENSITY;
	if (hard_resample)
		init_states |= DF_WITH_RESAMPLE;
	if (with_smooth)
		init_states |= DF_SMOOTHED;
	if (with_deshadow)
		init_states |= DF_DESHADOWED;
	if (enable_alarm_msg)
		init_states |= EF_ENABLE_ALARM_MSG;

	HParser parsers[MAX_LIDARS];
	PubHub *hubs[MAX_LIDARS] = {NULL};
	for (int i = 0; i < lidar_count; i++)
	{
		parsers[i] = ParserOpen(raw_bytes, device_ability, init_states, init_rpm, resample_res,
								with_chk, dev_id);
		hubs[i] = new PubHub;
		hubs[i]->nfan = 0;
		pthread_mutex_init(&hubs[i]->mtx, NULL);
	}

	if (g_type == "uart")
	{
		int *rates = new int[rate_list.size() + 1];
		for (int i = 0; i < rate_list.size(); i++)
		{
			rates[i] = rate_list[i];
			printf("[%d] => %d\n", i, rate_list[i]);
		}
		rates[rate_list.size()] = 0;
		g_reader = StartUartReader(port.c_str(), baud_rate, rates, parsers[0], hubs[0], Savelog, logPath);
	}
	else if (g_type == "udp")
	{
		LidarInfo lidars[MAX_LIDARS];
		for (int i = 0; i < lidar_count; i++)
		{
			lidars[i].parser = parsers[i];
			lidars[i].pub = hubs[i];
			strcpy(lidars[i].lidar_ip, lidar_ips[i].c_str());
			lidars[i].lidar_port = lidar_ports[i];
		}
		g_reader = StartUDPReader(local_port, is_group_listener, group_ip.c_str(), lidar_count, lidars, Savelog, logPath);
	}
	else if (g_type == "tcp")
	{
		g_reader = StartTCPReader(lidar_ips[0].c_str(), lidar_ports[0], parsers[0], hubs[0]);
	}

	while (ros::ok())
	{
		ros::spinOnce();
		bool idle = true;
		for (int i = 0; i < lidar_count; i++)
		{
			RawData *fans[MAX_FANS] = {NULL};

			if (!output_360)
			{
				if (GetFan(hubs[i], with_soft_resample, resample_res, fans))
				{
					if (output_scan)
					{
						PublishLaserScanFan(laser_pubs[i], fans[0], frame_id,
											min_dist, max_dist,
											with_angle_filter, min_angle, max_angle,
											inverted, reversed, zero_shift,
											custom_masks);
						// printf("free %x\n", fans[0]);
					}
					delete fans[0];
					idle = false;
				}
			}
			else
			{
				uint32_t ts_beg[2], ts_end[2];
				int n = GetAllFans(hubs[i], with_soft_resample, resample_res, fans, from_zero, ts_beg, ts_end);
				if (n > 0)
				{
					idle = false;
					if (output_scan)
					{
						PublishLaserScan(laser_pubs[i], n, fans, frame_id, min_dist, max_dist,
										 with_angle_filter, min_angle, max_angle,
										 inverted, reversed, zero_shift, from_zero,
										 ts_beg, ts_end, custom_masks);
					}

					if (output_cloud)
					{
						PublishCloud(cloud_pubs[i], n, fans, frame_id, max_dist,
									 with_angle_filter, min_angle, max_angle);
					}

					for (int i = 0; i < n; i++)
						delete fans[i];
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
