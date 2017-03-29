#ifndef _LOG_H_
#define _LOG_H_
#include "stdio.h"
#include "hal_uart.h"

#define CRITICAL    0X00
#define ERROR       0X01
#define WARN        0X02
#define INFO        0X03
#define DUMP        0X04

#define LOG_LEVEL   INFO

#define devDump(format, args...) printf( format, ##args )

#define logDump     devDump
#define assert()    devDump("[func:%s][line:%d]param is illegal.\r\n", __FUNCTION__, __LINE__)
#define log( level,format, args...)  \
{\
    if(LOG_LEVEL>=level ) \
        {\
            devDump( "[Dev    ] "format, ##args );\
        }\
}
#define cc1101Log( level,format, args...)  \
{\
    if(LOG_LEVEL>=level ) \
        {\
            devDump( "[CC1101 ] "format, ##args );\
        }\
}

#endif