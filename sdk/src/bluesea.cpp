#include "../include/bluesea.h"
BlueSeaLidarDriver::BlueSeaLidarDriver()
{
	m_reader=NULL;
	m_should_start=true;
	memset(&m_cmdlist,0,sizeof(CommandList));
}

BlueSeaLidarDriver::~BlueSeaLidarDriver()
{
	
}
void BlueSeaLidarDriver::getInitCmds(ArgData &argdata)
{
	m_argdata =argdata;
	if (argdata.uuid >= 0)
		sprintf(m_cmdlist.uuid, "LUUIDH");
	if (argdata.model >= 0)
		sprintf(m_cmdlist.model, "LTYPEH");
	if (argdata.rpm >= 0)
		sprintf(m_cmdlist.rpm, "LSRPM:%dH", argdata.rpm);
	if (argdata.resample >= 0)
	{

		if (argdata.resample > 0 && argdata.resample < 1)
			sprintf(m_cmdlist.res, "LSRES:%dH", (int)(argdata.resample * 1000));
		else
			sprintf(m_cmdlist.res, "LSRES:%dH", (int)argdata.resample);
	}
	if (argdata.with_smooth >= 0)
	{
		if (argdata.type == "uart")
			sprintf(m_cmdlist.smooth, "LSSS%dH", argdata.with_smooth);
		else
			sprintf(m_cmdlist.smooth, "LSSMT:%dH", argdata.with_smooth);
	}
	if (argdata.with_deshadow >= 0)
	{
		if (argdata.type == "uart")
			sprintf(m_cmdlist.fitter, "LFFF%dH", argdata.with_deshadow);
		else
			sprintf(m_cmdlist.fitter, "LSDSW:%dH", argdata.with_deshadow);
	}
	if (argdata.alarm_msg >= 0)
		sprintf(m_cmdlist.alarm, "LSPST:%dH", argdata.alarm_msg ? 3 : 1);
	if (argdata.direction >= 0)
		sprintf(m_cmdlist.direction, "LSCCW:%dH", argdata.direction);
	if (argdata.unit_is_mm >= 0)
		sprintf(m_cmdlist.unit_mm, "%s", argdata.unit_is_mm ? "LMDMMH" : "LMDCMH");
	if (argdata.with_confidence >= 0)
		sprintf(m_cmdlist.confidence, "%s", argdata.with_confidence ? "LOCONH" : "LNCONH");

	if (argdata.ats >= 0)
	{
		if (argdata.type == "vpc" || argdata.type == "uart")
			sprintf(m_cmdlist.ats, "LSATS:002H");
		else if (argdata.type == "udp")
			sprintf(m_cmdlist.ats, "LSATS:001H");
	}
}

void BlueSeaLidarDriver::openLidarThread()
{
	for (int i = 0; i < m_argdata.num; i++)
	{
		m_parsers[i] = ParserOpen(m_argdata.raw_bytes, true, m_argdata.dev_id, m_argdata.custom.error_circle, m_argdata.custom.error_scale, m_argdata.from_zero, m_argdata.time_mode,m_cmdlist, (char *)m_argdata.connectargs[i].arg1.c_str(), m_argdata.connectargs[i].arg2);
		m_hubs[i] = new PubHub;
		m_hubs[i]->nfan = 0;
		m_hubs[i]->offsetangle = -1;
		pthread_mutex_init(&m_hubs[i]->mtx, NULL);
	}
	if (m_argdata.type == "uart" || m_argdata.type == "vpc")
	{
		m_reader = StartUartReader(m_argdata.type.c_str(), m_argdata.connectargs[0].arg1.c_str(), m_argdata.connectargs[0].arg2, m_parsers[0], m_hubs[0]);
	}
	else if (m_argdata.type == "udp")
	{
		LidarInfo lidars[MAX_LIDARS];
		for (int i = 0; i < m_argdata.num; i++)
		{
			lidars[i].parser = m_parsers[i];
			lidars[i].pub = m_hubs[i];
			strcpy(lidars[i].lidar_ip, m_argdata.connectargs[i].arg1.c_str());
			lidars[i].lidar_port = m_argdata.connectargs[i].arg2;
		}
		m_reader = StartUDPReader(m_argdata.type.c_str(), m_argdata.localport, m_argdata.custom.is_group_listener, m_argdata.custom.group_ip.c_str(), m_argdata.num, lidars);
	}
	else if (m_argdata.type == "tcp")
	{
		m_reader = StartTCPReader(m_argdata.connectargs[0].arg1.c_str(), m_argdata.connectargs[0].arg2, m_parsers[0], m_hubs[0]);
	}
}

bool BlueSeaLidarDriver::sendCmd(int len, char *cmd, int index)
{
	if(strcmp(cmd,"LSTARH")==0)
		m_should_start=true;
	if(strcmp(cmd,"LSTOPH")==0)
		m_should_start=false;

	if (m_argdata.type == "uart")
	{
		return SendUartCmd(m_reader, len, cmd);
	}
	else if (m_argdata.type == "vpc")
	{
		return SendVpcCmd(m_reader, len, cmd);
	}
	else if (m_argdata.type == "udp")
	{
		if (index < 0)
		{
			int tmp = 0;
			memcpy(&tmp, m_reader, sizeof(int));
			for (int i = 0; i < tmp; i++)
			{
				SendUdpCmd(m_reader, i, len, cmd);
			}
			return true;
		}
		else
			return SendUdpCmd(m_reader, index, len, cmd);
	}
	else if (m_argdata.type == "tcp")
	{
		return SendTcpCmd(m_reader, len, cmd);
	}
	return false;
}
bool BlueSeaLidarDriver::sendUdpCmd2(char *ip, int len, char *cmd)
{
	return SendUdpCmd2(m_reader,ip, len, cmd);
}
PubHub*BlueSeaLidarDriver::getHub(int i)
{
	return m_hubs[i];
}
Parser*BlueSeaLidarDriver::getParser(int i)
{
	return m_parsers[i];
}
bool BlueSeaLidarDriver::checkIsRun(int index)
{
	PubHub *hub = m_hubs[index];
	int checkAngle = (m_argdata.from_zero == true ? 0 : 180);
	for (int i = 0; i < hub->nfan; i++)
	{
		int idx = getFirstidx(*(hub->fans[i]), checkAngle);
		if (idx >= 0)
		{
			hub->offsetangle = hub->fans[i]->angle;
			hub->offsetidx = idx;
			m_parsers[index]->isrun=true;
			DEBUG("lidar start work,offset angle %d offset idx %d raw_bytes:%d\n", hub->offsetangle/10, hub->offsetidx,m_parsers[index]->raw_mode);
			return true;
		}
	}
	return false;
}

int BlueSeaLidarDriver::GetAllFans(PubHub* pub, ArgData argdata, int8_t &counterclockwise)
{
	PubHub *hub = (PubHub *)pub;
	RawData *fans[MAX_FANS];
	// int checkAngle = (from_zero == 0 ? 0 : 180);
	//  解析出来一帧的数据
	pthread_mutex_lock(&hub->mtx);
	int cnt = 0, nfan = 0;
	for (int i = 1; i < hub->nfan; i++)
	{
		//DEBUG("%d %d",hub->fans[i]->angle,hub->offsetangle);
		if (hub->fans[i]->angle == hub->offsetangle)
		{
			hub->ts_end[0] = hub->fans[i]->ts[0];
			hub->ts_end[1] = hub->fans[i]->ts[1];
			cnt = i;
			break;
		}
	}
	if (cnt > 0)
	{
		// 起始点位在扇区的中间部分，删除扇区时就需要多保存一个扇区
		if (hub->offsetidx > 0)
		{
			nfan = cnt + 1;
			for (int i = 0; i <= cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];

			hub->nfan -= cnt;
		}
		else
		{
			nfan = cnt;
			for (int i = 0; i < cnt; i++)
			{
				fans[i] = hub->fans[i];
			}
			for (int i = cnt; i < hub->nfan; i++)
				hub->fans[i - cnt] = hub->fans[i];
			hub->nfan -= cnt;
		}
	}
	pthread_mutex_unlock(&hub->mtx);
	// 一帧数据的丢包检测以及软采样角分辨率的适配
	if (cnt > 0)
	{
		counterclockwise = fans[0]->counterclockwise;
		//如果是-1，则根据类型来区分，串口款都是逆时针，网络和虚拟串口默认为顺时针
		if(counterclockwise==-1)
		{
			if (argdata.type == "vpc" || argdata.type == "udp")
				counterclockwise=0;
			else if (argdata.type == "uart")
				counterclockwise=1;
		}
		bool circle = true;
		int total = fans[0]->span;
		hub->ts_beg[0] = fans[0]->ts[0];
		hub->ts_beg[1] = fans[0]->ts[1];

		for (int i = 0; i < nfan - 1; i++)
		{
			if ((fans[i]->angle + fans[i]->span) % 3600 != fans[i + 1]->angle)
			{
				circle = false;
			}
			total += fans[i + 1]->span;
		}
		//DEBUG("%d  %d %d \n", circle,total,hub->offsetidx);
		if (!circle || (total != 3600 && hub->offsetidx == 0) || ((total != 3600 + fans[0]->span) && hub->offsetidx > 0))
		{
			// clean imcomplent datas
			for (int i = 0; i < cnt; i++)
				delete fans[i];
			cnt = 0;
		}
	}
	if (cnt > 0)
	{
		if (argdata.soft_resample && argdata.resample > 0)
		{
			for (int i = 0; i < cnt; i++)
			{
				int NN = fans[i]->span / (10 * argdata.resample);
				if (NN < fans[i]->N)
				{
					// DEBUG("%s %d  %d,NN:%d,fans[i]->N:%d  %lf\n",__FUNCTION__,__LINE__,cnt,NN,fans[i]->span,resample_res);
					resample(fans[i], NN);
				}
				else if (NN > fans[i]->N)
				{
					//DEBUG("fan [%d] %d less than %d\n", i, fans[i]->N, NN);
					ERROR((int)SAMPLE_TOO_SMALL,Error::GetErrorString(SAMPLE_TOO_SMALL).c_str());
				}
			}
		}
		// 将扇区数据转为点 列表

		for (int j = hub->offsetidx; j < fans[0]->N; j++)
		{
			hub->consume.push_back(fans[0]->points[j]);
		}
		for (int i = 1; i < cnt; i++)
		{
			for (int j = 0; j < fans[i]->N; j++)
			{
				hub->consume.push_back(fans[i]->points[j]);
			}
		}
		for (int j = 0; j < hub->offsetidx; j++)
		{
			hub->consume.push_back(fans[cnt]->points[j]);
		}
		for (int i = 0; i < cnt; i++)
			delete fans[i];

		if (cnt > 0)
		{
			bool res = checkZeroDistance(hub->consume, argdata.custom.error_scale);
			if (res)
				hub->error_num = 0;
			else
			{
				hub->error_num++;
				if (hub->error_num >= argdata.custom.error_circle)
				{
					//DEBUG("There are many points with a distance of 0 in the current lidar operation");
					ERROR((int)DISTANCE_ZERO_LARGE,Error::GetErrorString(DISTANCE_ZERO_LARGE).c_str());
					hub->error_num = 0;
				}
			}
		}

		int N = hub->consume.size();
		double angle_increment = M_PI * 2 / N;
		Fitter *fitter = &argdata.fitter;
		if (fitter->isopen)
			filter(hub->consume, fitter->type, fitter->max_range*1000, fitter->min_range*1000, fitter->max_range_difference*1000, fitter->filter_window, angle_increment);
	
		counterclockwise=true;
		m_counterclockwise=counterclockwise;
		//DEBUG("%d",m_counterclockwise);
	}
	return cnt;
}
bool BlueSeaLidarDriver::GetFan(PubHub* pub, bool with_resample, double resample_res, RawData **fans)
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
			//DEBUG("fan %d less than %d\n", fans[0]->N, NN);
			ERROR((int)SAMPLE_TOO_SMALL,Error::GetErrorString(SAMPLE_TOO_SMALL).c_str());

		}
	}
	return got;
}
double BlueSeaLidarDriver::ROSAng(double ang)
{
	if(m_counterclockwise+m_argdata.reversed!=1)
	{
		return ang>180 ? 360-ang : -ang;
	}
	else
	{
		return ang>180 ? ang-360 :ang;
	}
}

int BlueSeaLidarDriver::GetCount(std::vector<DataPoint> data, double min_deg, double max_deg, double &min_pos, double &max_pos)
{
	int N = 0;

	for (int i = 0; i < data.size(); i++)
	{
		double deg = ROSAng(data[i].degree);

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

	//printf("angle filter [%f, %f] %d to %d, [%f, %f]\n", min_deg, max_deg, 1, N, min_pos, max_pos);
	return N;
}