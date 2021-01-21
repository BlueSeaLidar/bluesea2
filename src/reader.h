#ifndef _READER_
#define _READER_

#include "parser.h"

typedef void* HReader;
typedef void* HPublish;


void PublishData(HPublish, int, RawData**);;


HReader StartUartReader(const char* port, int baudrate, int* rate_list, HParser, HPublish);
bool SendUartCmd(HReader, int len, char*);


HReader StartUDPReader(const char* port, int baudrate, HParser, HPublish);
bool SendUDPCmd(HReader, int len, char*);


#endif
