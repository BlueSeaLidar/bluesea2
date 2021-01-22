#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "parser.h"

#define HDR_SIZE 6
#define HDR2_SIZE 8
#define HDR3_SIZE 16
#define HDR7_SIZE 28 
#define BUF_SIZE 8*1024

struct Parser 
{
	bool stream_mode;
	int rest_len;
	unsigned char rest_buf[BUF_SIZE];
	uint32_t flags;
	uint32_t init_flags;
	double resample_res;
	int with_chk;
	int raw_mode;
};

struct RawDataHdr
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
};

struct RawDataHdr2
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
};

struct RawDataHdr3
{
	unsigned short code;
	unsigned short N;
	unsigned short angle;
	unsigned short span;
	unsigned short fbase;
	unsigned short first;
	unsigned short last;
	unsigned short fend;
};

short LidarAng2ROS(short ang)
{
	ang = -ang;
	return ang < -1800 ? ang + 3600 : ang;
}

static uint32_t update_flags(unsigned char* buf)
{
	uint32_t v;
	memcpy(&v, buf, 4);
	return v;
}

static unsigned char is_data(unsigned char* buf)
{
	if (buf[1] != 0xfa)
		return 0;
		
	if (buf[0] == 0xce || buf[0] == 0xcf || buf[0] == 0xdf || buf[0] == 0xc7)
		return buf[0];

	return 0;
}

static RawData* GetData0xCE_2(const RawDataHdr& hdr, unsigned char* buf, uint32_t flags, int with_chk)
{
	RawData* dat = new RawData;
	memcpy(dat, &hdr, HDR_SIZE);
	dat->span = 360;

	int is_mm = flags & DF_UNIT_IS_MM;
	int with_conf = flags & DF_WITH_INTENSITY;

	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	unsigned char* pdat = buf+HDR_SIZE;
	for (int i=0; i<hdr.N; i++)
	{
		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short val = (v2<<8) | v;

		if (with_conf)
		{
			dat->points[i].confidence = val >> 13;
			dat->points[i].distance = val & 0x1fff;
			if (is_mm == 0) 
				dat->points[i].distance *= 10;
		} else {
			dat->points[i].confidence = is_mm ? val : val*10;
			dat->points[i].confidence = 0;
		}

		//dat->points[i].angle = hdr.angle*10 + 3600 * i / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (dat->span * i) / (10.0 * hdr.N);

		sum += val;
	}

	memcpy(&chk, buf+HDR_SIZE+hdr.N*2, 2);

	if (with_chk != 0 && chk != sum) 
	{
		delete dat;
		printf("chksum error");
		return NULL;
	}

	dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);

	return dat;
}


static RawData* GetData0xCE_3(const RawDataHdr& hdr, unsigned char* buf, uint32_t flags, int with_chk)
{
	RawData* dat = new RawData;
	memcpy(dat, &hdr, HDR_SIZE);
	int span = (flags & DF_FAN_90) ? 90 : 360;

	if (hdr.angle == 3420 && (flags & DF_FAN_90))
		span = 180;
	dat->span = span;

	unsigned char* pdat = buf+HDR_SIZE;
	// calc checksum
	unsigned short sum = hdr.angle + hdr.N, chk;
	for (int i=0; i<hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2<<8) | v;
		sum += vv;
		dat->points[i].distance = vv;
		//dat->points[i].angle = hdr.angle*10 + span * i * 10 / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum) 
	{
		delete dat;
		printf("chksum ce error");
		return NULL;;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get3 %d(%d)\n", hdr.angle, hdr.N);
	
	dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static RawData* GetData0xCF(const RawDataHdr2& hdr, unsigned char* pdat, int with_chk)
{
	RawData* dat = new RawData;
	memcpy(dat, &hdr, HDR2_SIZE);

	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	pdat += HDR2_SIZE;

	for (int i = 0; i < hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;

		unsigned short vv = (v2 << 8) | v;

		sum += vv;
		dat->points[i].distance = vv;
		//dat->points[i].angle = hdr.angle*10 + hdr.span * i *10 / hdr.N;
		dat->points[i].degree = hdr.angle/10.0 + (hdr.span * i) / (10.0 * hdr.N);
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		delete dat;
		printf("chksum cf error");
		return NULL;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get CF %d(%d) %d\n", hdr.angle, hdr.N, hdr.span);

	dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;
}

static RawData* GetData0xDF(const RawDataHdr3& hdr, unsigned char* pdat, int with_chk)
{
	RawData* dat = new RawData;
	memcpy(dat, &hdr, HDR3_SIZE);
	
	unsigned short sum = hdr.angle + hdr.N + hdr.span, chk;

	sum += hdr.fbase;
	sum += hdr.first;
	sum += hdr.last;
	sum += hdr.fend;

	double dan = (hdr.last - hdr.first) / double(hdr.N - 1);

	pdat += HDR3_SIZE;

	for (int i = 0; i < hdr.N; i++)
	{
		dat->points[i].confidence = *pdat++;
		sum += dat->points[i].confidence;

		unsigned short v = *pdat++;
		unsigned short v2 = *pdat++;
		unsigned short vv = (v2 << 8) | v;
		sum += vv;
		dat->points[i].distance = vv;
		//dat->points[i].angle = hdr.first + dan * i;
		dat->points[i].degree = (hdr.first + dan * i) / 100.0;
	}

	memcpy(&chk, pdat, 2);

	if (with_chk != 0 && chk != sum)
	{
		delete dat;
		printf("chksum df error");
		return NULL;
	}

	//memcpy(dat.data, buf+idx+HDR_SIZE, 2*hdr.N);
	//printf("get DF %d=%d %d %d\n", hdr.angle, hdr.first, hdr.N, hdr.span);

	dat->ros_angle = LidarAng2ROS(dat->angle + dat->span);
	return dat;

}

static int MsgProc(Parser* parser, int len, unsigned char* buf)
{
	if (len >= 8 && buf[0] == 'S' && buf[1] == 'T' && buf[6] == 'E' && buf[7] == 'D')
       	{
		parser->flags = update_flags(buf+2);
	}
	else {
		printf("unkown %d bytes %02x %02x %02x %02x\n", len, buf[0], buf[1], buf[2], buf[3]);

	}

	return -1;
}


static int ParseStream(Parser* parser, int len, unsigned char* buf, int* nfan, RawData* fans[]) 
{
	int max_fan = *nfan;
	int idx = 0;
	*nfan = 0;

	int unk = 0;
	unsigned char unknown[512];

	while (idx < len-32 && *nfan < max_fan)
	{
		unsigned char type = is_data(buf+idx);
		if (type == 0)
		{
			unknown[unk++] = buf[idx++];
			if (unk == sizeof(unknown)) 
			{
				printf("drop %d bytes : %02x %02x %02x %02x ...\n", unk, 
						unknown[0], unknown[1], 
						unknown[2], unknown[3]);
				unk = 0;
			}
			continue;
		}

		if (unk > 0) 
		{
			MsgProc(parser, unk, unknown);
			unk = 0;
		}

		RawDataHdr hdr;
		memcpy(&hdr, buf+idx, HDR_SIZE); 
		
		if (hdr.N > MAX_POINTS || hdr.N < 10) 
		{
			printf("points number %d seem not correct\n", hdr.N);
			idx += 2;
			continue;
		}

		if (type == 0xCE)
		{
			if (parser->raw_mode == 2) 
			{ 
				if (idx + HDR_SIZE + hdr.N*2+ 2 > len)
				{
					// need more bytes
					break;
				}
				
				RawData* fan = GetData0xCE_2(hdr, buf+idx,  parser->flags, parser->with_chk);
				if ( fan) 
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}

				idx += HDR_SIZE + hdr.N*2 + 2;
			}
			else {
				if (idx + HDR_SIZE + hdr.N*3 + 2 > len)
				{
					// need more bytes
					break;
				}
				RawData* fan = GetData0xCE_3(hdr, buf+idx, parser->flags, parser->with_chk);
				if (fan) 
				{
					fans[*nfan] = fan;
					*nfan += 1;
				}
				idx += HDR_SIZE + hdr.N*3 + 2;
			}
			
		}
		else if (type == 0xCF) {
			if (idx + hdr.N*3+ HDR2_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr2 hdr2;
			memcpy(&hdr2, buf+idx, HDR2_SIZE);

			RawData* fan = GetData0xCF(hdr2, buf+idx, parser->with_chk);
			if (fan)
			{
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*3+ HDR2_SIZE + 2;
		}
		else if (type == 0xDF) {
			if (idx + hdr.N*3+ HDR3_SIZE + 2 > len)
			{
				// need more bytes
				break;
			}
		       	RawDataHdr3 hdr3;
			memcpy(&hdr3, buf+idx, HDR3_SIZE);

			RawData* fan = GetData0xDF(hdr3, buf+idx, parser->with_chk);
			if (fan)
			{
				//printf("set [%d] %d\n", *nfan, fan->angle);
				fans[*nfan] = fan;
				*nfan += 1;
			}
			idx += hdr.N*3 + HDR3_SIZE + 2;
		} else {
			// should not 
		}
	} 
	return idx;
}


HParser ParserOpen(int raw_bytes, uint32_t init_flags, double resample_res, int with_chksum)
{
	Parser* parser = new Parser;

	parser->rest_len = 0;
	parser->raw_mode = raw_bytes;
	parser->init_flags = init_flags;
	parser->flags = init_flags;
	parser->resample_res = resample_res;

	return parser;
}

int ParserClose(HParser hP)
{
	Parser* parser = (Parser*)hP;
	delete parser;
	return 0;
}



int ParserRunStream(HParser hP, int len, unsigned char* bytes, RawData* fans[]) 
{
	Parser* parser = (Parser*)hP;

	int nfan = MAX_FANS;

	unsigned char* buf = new unsigned char[len + parser->rest_len];
	if (!buf) {
		printf("out of memory\n");
		return -1;
	}

	if (parser->rest_len > 0) {
		memcpy(buf, parser->rest_buf, parser->rest_len);
	}
	memcpy(buf+parser->rest_len, bytes, len);
	len += parser->rest_len;

	int used = ParseStream(parser, len, buf,  &nfan, fans);

#if 0
	for (int i=0; i<nfan; i++) 
	{
		fans[i]->ros_angle = LidarAng2ROS(fans[i]->angle + fans[i]->span);
	}
#endif

	parser->rest_len = len - used;
	if (parser->rest_len > 0) {
		memcpy(parser->rest_buf, buf+used, parser->rest_len);
	}

	delete buf;

	return nfan;
}

int ParserRun(HParser hP, int len, unsigned char* buf, RawData* fans[]) 
{	
	Parser* parser = (Parser*)hP;

	uint8_t type = buf[0];

	if (type == 0xCE)
	{
		RawDataHdr hdr;
		memcpy(&hdr, buf, HDR_SIZE); 

		if (HDR_SIZE + hdr.N*2+ 2 == len)
		{
			RawData* fan = GetData0xCE_2(hdr, buf,  parser->flags, parser->with_chk);
			if ( fan) 
			{
				fans[0] = fan;
				return 1;
			}
		}
		else if (HDR_SIZE + hdr.N*3 + 2 == len)
		{
			RawData* fan = GetData0xCE_3(hdr, buf, parser->flags, parser->with_chk);
			if (fan) 
			{
				fans[0] = fan;
				return 1;
			}
		}
		else printf("CE len %d N %d\n", len, hdr.N);
		
	}
	else if (type == 0xCF) 
	{
		RawDataHdr2 hdr;
		memcpy(&hdr, buf, HDR2_SIZE);

		if (hdr.N*3+ HDR2_SIZE + 2 > len)
		{
			// need more bytes
			printf("CF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0xCF(hdr, buf, parser->with_chk);
		if (fan)
		{
			//printf("CF %d + %d %d\n", fan->angle, fan->span, fan->N);
			fans[0] = fan;
			return 1;
		}
	}
	else if (type == 0xDF) 
	{
		RawDataHdr3 hdr;
		memcpy(&hdr, buf, HDR3_SIZE);

		if (hdr.N*3+ HDR3_SIZE + 2 == len)
		{
			// need more bytes
			printf("DF len %d N %d\n", len, hdr.N);
			return 0;
		}

		RawData* fan = GetData0xDF(hdr, buf, parser->with_chk);
		if (fan)
		{
			//printf("set [%d] %d\n", *nfan, fan->angle);
			fans[0] = fan;
			return 1;
		}
	}

	printf("skip packet %08x len %d\n", *(uint32_t*)buf, len);
	return 0;
}


char g_uuid[32] = "";

bool ParserScript(HParser hP, Script script, void* hnd)
{
	Parser* parser = (Parser*)hP;

	//read device's UUID 
	char buf[32];
	for (int i=0; i<10; i++)
	{
		if (script(hnd, 6, "LUUIDH", 12, "PRODUCT SN: ", 9, g_uuid))
		{
			printf("get product SN : \'%s\'\n", g_uuid);
			break;
		}
	}

	// setup output data format
	for (int i=0; i<10; i++) 
	{
		const char* cmd = (parser->init_flags & DF_UNIT_IS_MM) ?  "LMDMMH" : "LMDCMH"; 
		if (script(hnd, 6, cmd, 10, "SET LiDAR ", 9, buf) )
		{
			printf("set LiDAR unit to %s\n", buf);
			break;
		}
	}

	// enable/disable output intensity
	for (int i=0; i<10; i++) 
	{
		const char* cmd = (parser->init_flags & DF_WITH_INTENSITY) ? "LOCONH" : "LNCONH"; 
		if (script(hnd, 6, cmd, 6, "LiDAR ", 5, buf) )
		{
			printf("set LiDAR confidence to %s\n", buf);
			break;
		}
	}

	// set hard resample
	for (int i=0; i<10; i++) 
	{
		char cmd[32] = "LSRES:001H";
		if (parser->init_flags & DF_WITH_RESAMPLE)
		{
			sprintf(cmd, "LSRES:%03dH", int(parser->resample_res*1000));
		}

		char pattern[] = "set resolution ";
		if (script(hnd, strlen(cmd), cmd, strlen(pattern), pattern, 1, buf) )
		{
			printf("set resolution %s\n", buf);
			break;
		}
	}


	return true;
}
