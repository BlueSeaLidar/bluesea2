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
#include <bluesea2/DefenceZone.h>
// #include <dynamic_reconfigure/server.h>
// #include <bluesea2/DynParamsConfig.h>
#include <time.h>
#include <sys/stat.h>
#include <pthread.h>
#include <signal.h>
#include "reader.h"
#include "data_process.h"
HReader g_reader = NULL;
std::string g_type = "uart";
bool g_should_start = true;
// int g_offsetAngle;

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
	int error_num;
	int offsetangle;
};

void closeSignal(int sig)
{
	ROS_INFO("shutting down!");
	ros::shutdown();
	exit(1);
}

bool SendCmd(int len, char *cmd, int index)
{
	if (g_type == "uart")
	{
		return SendUartCmd(g_reader, len, cmd);
	}
	else if (g_type == "vpc")
	{
		return SendVpcCmd(g_reader, len, cmd);
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

		for (int i = 0; i < nr; i++)
			drop[skip++] = hub->fans[i];

		for (int i = nr; i < hub->nfan; i++)
		{
			hub->fans[i - nr] = hub->fans[i];
		}

		hub->nfan -= nr;
	}

	for (int i = 0; i < n; i++)
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
	if (hub->nfan > 0)
	{
		pthread_mutex_lock(&hub->mtx);
		fans[0] = hub->fans[0];
		for (int i = 1; i < hub->nfan; i++)
			hub->fans[i - 1] = hub->fans[i];
		hub->nfan--;
		got = true;
		pthread_mutex_unlock(&hub->mtx);
	}
	else
		return false;

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

int GetAllFans(HPublish pub, bool with_resample, double resample_res, RawData **fans, int collect_angle,
			   uint32_t *ts_beg, uint32_t *ts_end, int error_circle, double error_scale)
{
	PubHub *hub = (PubHub *)pub;
	// RawData* drop[MAX_FANS];
	pthread_mutex_lock(&hub->mtx);
	int cnt = 0;

	for (int i = 1; i < hub->nfan; i++)
	{
		if (hub->fans[i]->angle == hub->offsetangle * 10)
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
	// printf("%s %d  %d\n",__FUNCTION__,__LINE__,cnt);

	if (cnt > 0)
	{
		bool circle = true;
		int total = fans[0]->span;
		ts_beg[0] = fans[0]->ts[0];
		ts_beg[1] = fans[0]->ts[1];

		// std::cout<<"ts:"<<fans[0]->ts[0]<<" "<<fans[0]->ts[1]<<std::endl;
		// std::cout<<"ROS Time:"<<ros::Time::now()<<std::endl;

		for (int i = 0; i < cnt - 1; i++)
		{
			if ((fans[i]->angle + fans[i]->span) % 3600 != fans[i + 1]->angle)
			{
				circle = false;
			}
			total += fans[i + 1]->span;
			// printf("%d  %d %d \n", fans[i]->angle, fans[i]->span,fans[i + 1]->angle);
		}
		if (!circle || total != 3600)
		{
			printf("%d drop %d fans \n", total, cnt);
			// clean imcomplent datas
			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
		int sum = 0;
		int lengthZeroNum = 0;
		for (int i = 0; i < cnt; i++)
		{
			sum += fans[i]->N;
			for (int j = 0; j < fans[i]->N; j++)
			{
				if (fans[i]->points[j].distance == 0)
				{
					lengthZeroNum++;
				}
			}
		}
		if (sum * error_scale < lengthZeroNum)
			hub->error_num++;

		if (hub->error_num >= error_circle)
		{
			printf("There are many points with a distance of 0 in the current lidar operation,sum:%d lengthZeroNum:%d\n", sum, lengthZeroNum);
			hub->error_num = 0;
		}
		// printf("123   %d  %d %d\n",sum,lengthZeroNum,hub->error_num);
	}
	if (cnt > 0)
	{
		if (with_resample&&resample_res>0)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * resample_res);
				if (NN < fans[i]->N)
				{
					//printf("%s %d  %d,NN:%d,fans[i]->N:%d  %lf\n",__FUNCTION__,__LINE__,cnt,NN,fans[i]->span,resample_res);
					resample(fans[i], NN);				}
				else if (NN > fans[i]->N)
				{
					printf("fan [%d] %d less than %d\n", i, fans[i]->N, NN);
				}
			}
		}
	}
	//printf("%s %d  %d\n",__FUNCTION__,__LINE__,cnt);
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
void getMSGData(sensor_msgs::LaserScan &msg, RawData *fan, bool reversed,
				double min_ang, double max_ang, double min_dist, double max_dist, bool with_filter, const std::vector<Range> &custom_masks, int i, int index)
{
	int start, end;
	if (i == -1)
	{
		start = 0;
		end = fan->N;
	}
	else if (i == 0)
	{
		if (reversed)
		{
			start = index;
			end = fan->N;
		}
		else
		{
			start = 0;
			end = index;
		}
	}
	else if (i == 1)
	{
		if (reversed)
		{
			start = 0;
			end = index;
		}
		else
		{
			start = index;
			end = fan->N;
		}
	}
	int N = 0;
	double min_deg = min_ang * 180 / M_PI;
	double max_deg = max_ang * 180 / M_PI;
	if (reversed)
	{
		for (int i = end - 1; i >= start; i--)
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
		for (int i = start; i < end; i++)
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
			printf(" %d %d  %d  %lf \n", fan->points[i].distance, fan->points[i].confidence, N, fan->points[i].degree);
			N++;
		}
	}
}
void PublishLaserScanFan(ros::Publisher &laser_pub, RawData *fan,
						 std::string &frame_id,
						 double min_dist, double max_dist,
						 bool with_filter, double min_ang, double max_ang,
						 bool inverted, bool reversed, double zero_shift, bool from_zero,
						 const std::vector<Range> &custom_masks, int collect_angle)
{
	int N = fan->N;

	if (zero_shift > M_PI || zero_shift < -M_PI)
	{
		int num = zero_shift / M_PI;
		zero_shift = zero_shift - num * M_PI;
	}
	if (zero_shift != 0)
	{
		double zero_shift_tmp = zero_shift * 180 / M_PI;
		for (int i = 0; i < N; i++)
		{
			double deg = fan->points[i].degree + zero_shift_tmp;
			fan->points[i].degree = deg;
		}
	}
	bool isspecial = false;
	int index = 0;
	int tmp_angle = from_zero ? 0 : 1800;
	if (from_zero)
	{
		if (fan->angle < 3600 && (fan->angle + fan->span) > 3600)
		{
			index = (3600 - fan->angle) * 1.0 / fan->span * fan->N;
			isspecial = true;
		}
		else
			index = -1;
	}
	else
	{
		if (fan->angle < 1800 && (fan->angle + fan->span) > 1800)
		{
			index = (1800 - fan->angle) * 1.0 / fan->span * fan->N;
			isspecial = true;
		}
		else
			index = -1;
	}
	RawData tmpdata[2];
	tmpdata[0].angle = fan->angle;
	tmpdata[0].span = fan->span * index / fan->N;
	tmpdata[0].N = index;

	tmpdata[1].angle = tmp_angle;
	tmpdata[1].span = fan->span * (fan->N - index) / fan->N;
	tmpdata[1].N = fan->N - index;

	// msg.header.stamp = ros::Time::now();
	sensor_msgs::LaserScan msg;
	msg.header.stamp.sec = fan->ts[0];
	msg.header.stamp.nsec = fan->ts[1];

	msg.header.frame_id = frame_id;

	double scan_time = 1 / 100.;
	msg.scan_time = scan_time;
	msg.time_increment = scan_time / fan->N;

	msg.range_min = min_dist;
	msg.range_max = max_dist; // 8.0;
	double min_pos, max_pos;
	// if (!isspecial)
	if (1)
	{
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
		// msg.angle_min = M_PI;
		// 	msg.angle_max = -M_PI + (2 * M_PI) / N;
		// 	msg.angle_increment = -M_PI * 2 / N;
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
		// printf(" %f %f  %d  %f %f\n", min_pos,max_pos,fan->angle);
		getMSGData(msg, fan, reversed, min_ang, max_ang, min_dist, max_dist, with_filter, custom_masks, -1, index);
		laser_pub.publish(msg);
		return;
	}
	else
	{
		for (int i = 0; i < 2; i++)
		{
			msg.intensities.resize(tmpdata[i].N); // fan->N);
			msg.ranges.resize(tmpdata[i].N);	  // fan->N);
			double min_pos, max_pos;

			if (inverted)
			{
				min_pos = ROSAng(-tmpdata[i].angle / 10) * 10 * M_PI / 1800;
				max_pos = -min_pos + tmpdata[i].span * M_PI / 1800;
				msg.angle_min = min_pos;
				msg.angle_max = max_pos;
				msg.angle_increment = (fan->span * M_PI / 1800) / fan->N;
			}
			else
			{
				min_pos = ROSAng(tmpdata[i].angle / 10) * 10 * M_PI / 1800;
				max_pos = min_pos - tmpdata[i].span * M_PI / 1800;
				msg.angle_min = max_pos;
				msg.angle_max = min_pos;
				msg.angle_increment = -(fan->span * M_PI / 1800) / fan->N;
			}
			getMSGData(msg, fan, reversed, min_ang, max_ang, min_dist, max_dist, with_filter, custom_masks, i, index);
			printf(" %d %d  %d %d \n", tmpdata[0].angle, tmpdata[0].span, tmpdata[1].angle, tmpdata[1].span);
			laser_pub.publish(msg);
			// printf(" %f %f  %d %d \n", min_pos,max_pos,index,i);
		}
	}
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
					  const std::vector<Range> &custom_masks, int collect_angle)
{
	sensor_msgs::LaserScan msg;
	int N = 0;
	if (zero_shift > M_PI || zero_shift < -M_PI)
	{
		int num = zero_shift / M_PI;
		zero_shift = zero_shift - num * M_PI;
	}

	double zero_shift_tmp = zero_shift * 180 / M_PI;
	for (int j = 0; j < nfan; j++)
	{
		N += fans[j]->N;
		if (zero_shift == 0)
			continue;
		for (int i = 0; i < fans[j]->N; i++)
		{
			double deg = fans[j]->points[i].degree + zero_shift_tmp;
			fans[j]->points[i].degree = deg;
		}
	}
	//printf("%d\n",N);
	// make  min_ang max_ang  convert to mask
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
	DataPoint datapoint[N] = {0};
	// splitting sectors
	int index = 0;
	bool first = true;
	if (from_zero)
	{
		if (collect_angle > 180)
		{
			index = (360 - collect_angle) / (fans[0]->span / 10.0 / fans[0]->N);
			first = true;
		}
		else if (collect_angle < 180 && collect_angle > 0)
		{
			index = (collect_angle - 0) / (fans[nfan - 1]->span / 10.0 / fans[nfan - 1]->N);
			first = false;
		}
		else if (collect_angle == 0)
		{
			index = 0;
			first = true;
		}
	}
	else
	{
		if (collect_angle > 180)
		{
			index = (collect_angle - 180) / (fans[nfan - 1]->span / 10.0 / fans[nfan - 1]->N);
			first = false;
		}
		else if (collect_angle <= 180)
		{
			index = (180 - collect_angle) / (fans[0]->span / 10.0 / fans[0]->N);
			first = true;
		}
	}
	int pointindex = 0;
	if (first)
	{
		for (int i = index; i < fans[0]->N; i++)
		{
			datapoint[pointindex] = fans[0]->points[i];
			pointindex++;
		}
		for (unsigned int i = 1; i < nfan; i++)
		{
			for (unsigned int j = 0; j < fans[i]->N; j++)
			{
				datapoint[pointindex] = fans[i]->points[j];
				pointindex++;
			}
		}
		for (int i = 0; i < index; i++)
		{
			datapoint[pointindex] = fans[0]->points[i];
			pointindex++;
		}
	}
	else
	{
		for (int i = index; i < fans[nfan - 1]->N; i++)
		{
			datapoint[pointindex] = fans[nfan - 1]->points[i];
			pointindex++;
		}
		for (unsigned int i = 0; i < nfan - 1; i++)
		{
			for (unsigned int j = 0; j < fans[i]->N; j++)
			{
				datapoint[pointindex] = fans[i]->points[j];
				pointindex++;
			}
		}
		for (int i = 0; i < index; i++)
		{
			datapoint[pointindex] = fans[nfan - 1]->points[i];
			pointindex++;
		}
	}
	int idx = 0;
	if (reversed)
	{
		for (int i = N - 1; i >= 0; i--)
		{
			double deg = ROSAng(datapoint[i].degree);
			double d = datapoint[i].distance / 1000.0;
			bool custom = false;
			for (int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min <= deg && deg <= custom_masks[k].max)
					custom = true;
			}

			if (datapoint[i].distance == 0 || d > max_dist || d < min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = datapoint[i].confidence;
			idx++;
		}
	}
	else
	{
		for (int i = 0; i <= N - 1; i++)
		{
			// printf(" %d %d  %d  %lf \n", datapoint[i].distance,datapoint[i].confidence,N,datapoint[i].degree);

			double deg = ROSAng(datapoint[i].degree);
			double d = datapoint[i].distance / 1000.0;
			bool custom = false;
			for (int k = 0; k < custom_masks.size() && !custom; k++)
			{
				if (with_filter && custom_masks[k].min <= deg && deg <= custom_masks[k].max)
					custom = true;
			}

			if (datapoint[i].distance == 0 || d > max_dist || d < min_dist || custom)
				msg.ranges[idx] = std::numeric_limits<float>::infinity();
			else
				msg.ranges[idx] = d;

			msg.intensities[idx] = datapoint[i].confidence;

			idx++;
		}
	}
	laser_pub.publish(msg);
}

void PublishCloud(ros::Publisher &cloud_pub, int nfan, RawData **fans, std::string &frame_id,
				  double max_dist,
				  bool with_filter, double min_ang, double max_ang, int collect_angle)
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

// void setup_params(bluesea2::DynParamsConfig &config, uint32_t level)
// {
// 	ROS_INFO("Change RPM to [%d]", config.rpm);

// 	char cmd[32];
// 	sprintf(cmd, "LSRPM:%dH", config.rpm);

// 	SendCmd(strlen(cmd), cmd, 0);
// }

// service call back function
bool stop_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	g_should_start = false;
	ROS_INFO("Stop motor  index:%ld", req.index);
	char cmd[] = "LSTOPH";
	return SendCmd(6, cmd, req.index);
}

// service call back function
bool start_motor(bluesea2::Control::Request &req, bluesea2::Control::Response &res)
{
	g_should_start = true;
	char cmd[] = "LSTARH";
	ROS_INFO("Start motor  index:%ld", req.index);
	return SendCmd(6, cmd, req.index);
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
		return SendUdpCmd2(g_reader, (char *)req.ip.c_str(), 12, cmd);
		;
	}
	return false;
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
void getCMDList(CommandList &cmdlist, std::string type, int uuid, int model,
				int init_rpm, double resample_res, int with_smooth, int with_deshadow,
				int enable_alarm_msg, int direction, int unit_is_mm, int with_confidence, int ats)
{

	if (uuid >= 0)
		sprintf(cmdlist.uuid, "LUUIDH");
	if (model >= 0)
		sprintf(cmdlist.model, "LTYPEH");
	if (init_rpm >= 0)
		sprintf(cmdlist.rpm, "LSRPM:%dH", init_rpm);
	if (resample_res >= 0)
	{
		
		if (resample_res > 0 && resample_res < 1)
			sprintf(cmdlist.res, "LSRES:%dH", (int)(resample_res * 1000));
		else
			sprintf(cmdlist.res, "LSRES:%dH", (int)resample_res);

	}
	if (with_smooth >= 0)
	{
		if (type == "uart")
			sprintf(cmdlist.smooth, "LSSS%dH", with_smooth);
		else
			sprintf(cmdlist.smooth, "LSSMT:%dH", with_smooth);
	}
	if (with_deshadow >= 0)
	{
		if (type == "uart")
			sprintf(cmdlist.fitter, "LFFF%dH", with_deshadow);
		else
			sprintf(cmdlist.fitter, "LSDSW:%dH", with_deshadow);
	}
	if (enable_alarm_msg >= 0)
		sprintf(cmdlist.alarm, "LSPST:%dH", enable_alarm_msg ? 3 : 1);
	if (direction >= 0)
		sprintf(cmdlist.direction, "LSCCW:%dH", direction);
	if (unit_is_mm >= 0)
		sprintf(cmdlist.unit_mm, "%s", unit_is_mm ? "LMDMMH" : "LMDCMH");
	if (with_confidence >= 0)
		sprintf(cmdlist.confidence, "%s", with_confidence ? "LOCONH" : "LNCONH");

	if (ats >= 0)
	{
		if (type == "vpc" || type == "uart")
			sprintf(cmdlist.ats, "LSATS:002H");
		else if (type == "udp")
			sprintf(cmdlist.ats, "LSATS:001H");
	}
}

int autoGetFirstAngle2(HPublish pub, bool from_zero)
{
	PubHub *hub = (PubHub *)pub;
	std::vector<RawData> raws;
	std::string result;
	int ret = -1;

	for (int i = 0; i < hub->nfan; i++)
	{
		//printf("angle %d  i:%d\n", hub->fans[i]->angle,i);
		ret = autoGetFirstAngle(*(hub->fans[i]), from_zero, raws, result);
		if (ret >= 0 || ret == -2)
		{
			hub->nfan = 0;
			break;
		}
	}
	return ret;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "bluesea2_laser_publisher");
	ros::NodeHandle node_handle;
	ros::NodeHandle priv_nh("~");
	signal(SIGINT, closeSignal);
	/*****************************ROS arg start************************************/
	std::string laser_topics[MAX_LIDARS];
	std::string cloud_topics[MAX_LIDARS];
	priv_nh.param("topic", laser_topics[0], std::string("scan"));
	priv_nh.param("cloud_topic", cloud_topics[0], std::string("cloud"));
	std::string frame_id;
	priv_nh.param("frame_id", frame_id, std::string("LH_laser")); // could be used for rviz

	/*****************************ROS arg end************************************/
	/*****************************DATA arg start************************************/
	// LiDAR comm type, could be "uart" or "udp"
	priv_nh.param("type", g_type, std::string("uart"));
	printf("type is %s\n", g_type.c_str());
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
	priv_nh.param("lidar_ip", lidar_ips[0], std::string("192.168.158.98"));

	int lidar_count = 1;

	for (int i = 1; i < MAX_LIDARS; i++)
	{
		char s[32], t[32];
		sprintf(s, "lidar%d_ip", i);
		priv_nh.param(s, lidar_ips[i], std::string(""));
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
	// for (int i = 0; i < lidar_count; i++)
	// {
	// 	printf("lidar[%d] address %s:%d, topic `%s\n", i,
	// 		   lidar_ips[i].c_str(), lidar_ports[i],
	// 		   laser_topics[i].c_str());
	// }

	bool is_group_listener;
	priv_nh.param("group_listener", is_group_listener, false);
	std::string group_ip;
	priv_nh.param("group_ip", group_ip, std::string("224.1.1.91"));

	// device identity in data packets, used when multiple lidars connect to single controller
	int dev_id;
	priv_nh.param("dev_id", dev_id, ANYONE); //

	// raw data format
	int raw_bytes, normal_size;
	bool with_chk;
	priv_nh.param("raw_bytes", raw_bytes, 3);		// packet mode : 2bytes or 3bytes
	priv_nh.param("normal_size", normal_size, -1);	// -1 : allow all packet, N : drop packets whose points less than N
	priv_nh.param("with_checksum", with_chk, true); // true : enable packet checksum

	// is lidar inverted
	bool inverted, reversed;
	priv_nh.param("inverted", inverted, false);
	priv_nh.param("reversed", reversed, false);

	// zero position
	double zero_shift;
	priv_nh.param("zero_shift", zero_shift, 0.0);

	// data output
	bool output_scan = true, output_cloud = false, output_360 = true;
	priv_nh.param("output_scan", output_scan, true);	// true: enable output angle+distance mode, 0: disable
	priv_nh.param("output_cloud", output_cloud, false); // false: enable output xyz format, 0 : disable
	priv_nh.param("output_360", output_360, true);		// true: packet data of 360 degree (multiple RawData), publish once
														// false: publish every RawData (36 degree)

	//  angle filter
	bool with_angle_filter;
	double min_angle, max_angle;
	priv_nh.param("with_angle_filter", with_angle_filter, false); // true: enable angle filter, false: disable
	priv_nh.param("min_angle", min_angle, -M_PI);				  // angle filter's low threshold, default value: -pi
	priv_nh.param("max_angle", max_angle, M_PI);				  // angle filters' up threashold, default value: pi

	// range limitation
	double min_dist, max_dist;
	priv_nh.param("min_dist", min_dist, 0.000);	 // min detection range, default value: 0M
	priv_nh.param("max_dist", max_dist, 9999.0); // max detection range, default value: 9999M
	if (inverted)
	{
		double tmp = min_angle * -1;
		min_angle = max_angle * -1;
		max_angle = tmp;
	}
	if (reversed)
	{
		double tmp = min_angle * -1;
		min_angle = max_angle * -1;
		max_angle = tmp;
	}
	std::vector<Range> custom_masks;
	Range range1, range2;
	if (min_angle < max_angle)
	{
		range1.min = -180;
		range1.max = min_angle / M_PI * 180;
		range2.min = max_angle / M_PI * 180;
		range2.max = 180;
		custom_masks.push_back(range1);
		custom_masks.push_back(range2);
		//printf("Visible range:%lf %lf %lf %lf\n", range1.min, range1.max, range2.min, range2.max);
	}

	for (int i = 1;; i++)
	{
		char name[32];
		sprintf(name, "mask%d", i);
		Range range;
		if (!get_range_param(priv_nh, name, range))
			break;
		if (inverted)
		{
			double tmp = range.min * -1;
			range.min = range.max * -1;
			range.max = tmp;
		}
		if (reversed)
		{
			double tmp = range.min * -1;
			range.min = range.max * -1;
			range.max = tmp;
		}
		double min_angle_tmp = min_angle / M_PI * 180;
		double max_angle_tmp = max_angle / M_PI * 180;

		if (min_angle_tmp <= range.min)
		{
			if (range.max <= max_angle_tmp)
			{
			}
			else
			{
				range.max = max_angle_tmp;
			}
		}
		else
		{
			if (range.max <= max_angle_tmp)
			{
				range.min = min_angle_tmp;
			}
			else
			{
				range.min = min_angle_tmp;
				range.max = max_angle_tmp;
			}
		}
		custom_masks.push_back(range);
		// printf("Invisible range:%lf %lf\n", range.min, range.max);
	}
	// output data format
	bool from_zero = false;
	priv_nh.param("from_zero", from_zero, false); // true : angle range [0 - 360), false: angle range [-180, 180)
	std::string logPathTmp;
	// Synthesize the full  log path
	priv_nh.param("logPath", logPathTmp, std::string("/tmp/log"));
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

	sprintf(logPath, "%s/log_%04d%02d%02d_%02d%02d%02d.txt", logPathTmp.c_str(), tm_now->tm_year + 1900, tm_now->tm_mon, tm_now->tm_mday, tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);

	int error_circle = 0;
	double error_scale = 0;
	priv_nh.param("error_circle", error_circle, 3);
	priv_nh.param("error_scale", error_scale, 0.9);

	/*****************************DATA arg end************************************/
	/*****************************GET arg start************************************/
	int uuid, model;
	priv_nh.param("uuid", uuid, -1);
	priv_nh.param("model", model, -1);
	/*****************************GET arg end************************************/
	/*****************************SET arg start************************************/
	int init_rpm;
	priv_nh.param("rpm", init_rpm, -1); // set motor RPM
	// angle composate
	bool hard_resample, with_soft_resample;
	double resample_res,resample_res2;
	priv_nh.param("hard_resample", hard_resample, false);	   // resample angle resolution
	priv_nh.param("soft_resample", with_soft_resample, false); // resample angle resolution
	priv_nh.param("resample_res", resample_res, -1.0); // resample angle resolution
	
	if (!hard_resample)
		resample_res2 = -1;	

	int with_smooth, with_deshadow;
	priv_nh.param("with_smooth", with_smooth, -1);	   // lidar data smooth filter
	priv_nh.param("with_deshadow", with_deshadow, -1); // data shadow filter
	int enable_alarm_msg;
	priv_nh.param("alarm_msg", enable_alarm_msg, -1); // let lidar upload alarm message
	int direction;
	priv_nh.param("direction", direction, -1);
	int unit_is_mm, with_confidence;
	priv_nh.param("unit_is_mm", unit_is_mm, -1); // 0 : distance is CM, 1: MM
	priv_nh.param("with_confidence", with_confidence, -1);
	int ats;
	priv_nh.param("ats", ats, -1);
	/*****************************SET arg end************************************/
	CommandList cmdlist;
	memset(&cmdlist, 0, sizeof(CommandList));
	getCMDList(cmdlist, g_type, uuid, model, init_rpm, resample_res2, with_smooth, with_deshadow, enable_alarm_msg, direction, unit_is_mm, with_confidence, ats);
	// Synthesize the full  log path
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
	ros::ServiceServer switchZone_srv = node_handle.advertiseService("switchZone_motor", switchZone_motor);

	// dynamic_reconfigure::Server<bluesea2::DynParamsConfig> server;
	// server.setCallback(boost::bind(&setup_params, _1, _2));

	HParser parsers[MAX_LIDARS];
	PubHub *hubs[MAX_LIDARS] = {NULL};
	for (int i = 0; i < lidar_count; i++)
	{
		parsers[i] = ParserOpen(raw_bytes, with_chk, dev_id, error_circle, error_scale,from_zero, logPath,cmdlist,(char*)lidar_ips[i].c_str(),lidar_ports[i]);
		hubs[i] = new PubHub;
		hubs[i]->nfan = 0;
		hubs[i]->offsetangle = -1;
		pthread_mutex_init(&hubs[i]->mtx, NULL);
	}

	if (g_type == "uart" || g_type == "vpc")
	{
		int *rates = new int[rate_list.size() + 1];
		for (int i = 0; i < rate_list.size(); i++)
		{
			rates[i] = rate_list[i];
		}
		rates[rate_list.size()] = 0;
		g_reader = StartUartReader(g_type.c_str(), port.c_str(), baud_rate, rates, parsers[0], hubs[0]);
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
		g_reader = StartUDPReader(g_type.c_str(), local_port, is_group_listener, group_ip.c_str(), lidar_count, lidars);
	}
	else if (g_type == "tcp")
	{
		g_reader = StartTCPReader(lidar_ips[0].c_str(), lidar_ports[0], parsers[0], hubs[0]);
	}
	// get offsetangle
	while (ros::ok())
	{
		ros::spinOnce();
		bool idle = true;
		int ret = -1;
		for (int i = 0; i < lidar_count; i++)
		{
			if (hubs[i]->offsetangle == -1)
			{
				ret = autoGetFirstAngle2(hubs[i], from_zero);
				if (ret >= 0)
				{
					hubs[i]->offsetangle = ret / 10;
					printf("start  offset angle %d\n", hubs[i]->offsetangle);
				}
				// printf("start  offset angle %d\n", hubs[i]->offsetangle);
				continue;
			}
			else
			{
				idle = false;
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
												inverted, reversed, zero_shift, from_zero,
												custom_masks, hubs[i]->offsetangle);
						}
						delete fans[0];
						idle = false;
					}
					// printf("%s %d\n",__FUNCTION__,__LINE__);
				}
				else
				{

					uint32_t ts_beg[2], ts_end[2];
					int n = GetAllFans(hubs[i], with_soft_resample, resample_res, fans, 0, ts_beg, ts_end, error_circle, error_scale);
					if (n > 0)
					{
						idle = false;
						if (output_scan)
						{
							PublishLaserScan(laser_pubs[i], n, fans, frame_id, min_dist, max_dist,
											 with_angle_filter, min_angle, max_angle,
											 inverted, reversed, zero_shift, from_zero,
											 ts_beg, ts_end, custom_masks, hubs[i]->offsetangle);
						}

						if (output_cloud)
						{
							PublishCloud(cloud_pubs[i], n, fans, frame_id, max_dist,
										 with_angle_filter, min_angle, max_angle, hubs[i]->offsetangle);
						}

						for (int i = 0; i < n; i++)
							delete fans[i];
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
