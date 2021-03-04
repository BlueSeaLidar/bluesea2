#include <stdio.h>
#include <stdlib.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <errno.h>
#include <sys/types.h> 
#include <netinet/in.h> 
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h> 
#include <arpa/inet.h> 
#include <stdarg.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h> 
#include <signal.h>
#include <pthread.h>

#include "reader.h"

struct CmdHeader
{
	unsigned short sign;
	unsigned short cmd;
	unsigned short sn;
	unsigned short len;
};

struct UDPInfo 
{
	HParser hParser;
	HPublish hPublish;
	int fd_udp;
	int listen_port;
	int lidar_port;
	char lidar_ip[32];
	char group_ip[32];
	pthread_t thr;
};

struct KeepAlive {
	uint32_t clock;
	uint32_t reserved[7];
};
	
// CRC32
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}



// 
bool send_cmd_udp_f(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const void* snd_buf, bool bpr)
{
	char buffer[2048];
	CmdHeader* hdr = (CmdHeader*)buffer;
	hdr->sign = 0x484c;
	hdr->cmd = cmd;
	hdr->sn = sn;

	len = ((len + 3) >> 2) * 4;

	hdr->len = len;

	memcpy(buffer + sizeof(CmdHeader), snd_buf, len);

	int n = sizeof(CmdHeader);
	unsigned int* pcrc = (unsigned int*)(buffer + sizeof(CmdHeader) + len);
	pcrc[0] = stm32crc((unsigned int*)(buffer + 0), len/4 + 2);

	sockaddr_in to;
	to.sin_family = AF_INET;
	to.sin_addr.s_addr = inet_addr(dev_ip);
	to.sin_port = htons(dev_port);

	int len2 = len + sizeof(CmdHeader) + 4;

	sendto(fd_udp, buffer, len2, 0, (struct sockaddr*)&to, sizeof(struct sockaddr));

	if (bpr) {
		char s[3096];
		for (int i = 0; i < len2; i++) 
			sprintf(s + 3 * i, "%02x ", (unsigned char)buffer[i]);

		printf("send to %s:%d 0x%04x sn[%d] L=%d : %s\n", 
				dev_ip, dev_port, cmd, sn, len, s);
	}
	return true;
}

bool send_cmd_udp(int fd_udp, const char* dev_ip, int dev_port,
	       	int cmd, int sn, 
		int len, const void* snd_buf)
{
	return send_cmd_udp_f(fd_udp, dev_ip, dev_port, cmd, sn, len, snd_buf, true);
}



void* UdpThreadProc(void* p)
{
	UDPInfo* info = (UDPInfo*)p;

	int fd_udp = info->fd_udp;

	struct KeepAlive alive;
	memset(&alive, 0, sizeof(alive));
	alive.clock = time(NULL);

	// acknowlege device 
	//int rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x4753, rand(), 0, NULL);
	int rt = send_cmd_udp_f(info->fd_udp, info->lidar_ip, info->lidar_port, 
			0x4b41, rand(), sizeof(alive), &alive, false);

	// send requirement to lidar
	char cmd[12] = "LGCPSH";
	rt = send_cmd_udp(fd_udp, info->lidar_ip, info->lidar_port, 0x0043, rand(), 6, cmd);

	RawData* fans[MAX_FANS];
	char buf[1024];


	while (1) 
	{
		fd_set fds;
		FD_ZERO(&fds); 

		FD_SET(fd_udp, &fds); 
	
		struct timeval to = { 5, 5 }; 
		int ret = select(fd_udp+1, &fds, NULL, NULL, &to); 

		if (ret == 0) {
			char cmd[12] = "LGCPSH";
			rt = send_cmd_udp(info->fd_udp, info->lidar_ip, info->lidar_port, 0x0043, rand(), 6, cmd);
		}
		
		if (ret < 0) {
			printf("select error\n");
			continue;
	       	}

		// read UDP data
		if (FD_ISSET(fd_udp, &fds)) 
		{ 
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			int nr = recvfrom(fd_udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0) {
				//printf("udp %02x%02x\n", buf[0], buf[1]);
				int nfan = ParserRun(info->hParser, nr, (uint8_t*)buf, &(fans[0]));
				//for (int i=0; i<nfan; i++)
				//	 printf("fan %x %d + %d\n", fans[i], fans[i]->angle, fans[i]->span);
				if (nfan > 0) {
					PublishData(info->hPublish, nfan, fans);
				}

				uint32_t now = time(NULL);
				if (now != alive.clock) 
				{
					alive.clock = now;
					send_cmd_udp_f(info->fd_udp, 
							info->lidar_ip, info->lidar_port, 
							0x4b41, rand(), 
							sizeof(alive), &alive, 
							false);
				}
			}
		}
	}
	return NULL;
}


HReader StartUDPReader(const char* lidar_ip, unsigned short lidar_port, 
		const char* group_ip, unsigned short listen_port, 
		HParser hParser, HPublish hPub)
{
	UDPInfo* info = new UDPInfo;
	info->hParser = hParser;
	info->hPublish = hPub;
	info->listen_port = listen_port;
	info->lidar_port = lidar_port;
	strcpy(info->lidar_ip, lidar_ip);
	strcpy(info->group_ip, group_ip);

	// open UDP port
	info->fd_udp  = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(listen_port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int rt = ::bind(info->fd_udp, (struct sockaddr *)&addr, sizeof(addr));
	if (rt != 0)
	{
		printf("bind port %d failed\n", listen_port);
	}
	
	printf("start udp %s:%d udp %d\n", lidar_ip, lidar_port, info->fd_udp);

	pthread_create(&info->thr, NULL, UdpThreadProc, info); 

	return info;
} 

bool SendUdpCmd(HReader hr, int len, char* cmd)
{
	UDPInfo* info = (UDPInfo*)hr;
	if (!info || info->fd_udp <= 0)
		return false;
		
	return send_cmd_udp(info->fd_udp, info->lidar_ip, info->lidar_port, 0x0043, rand(), len, cmd);
}


