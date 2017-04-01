#ifndef TYPEDEF_H_
#define TYPEDEF_H_

typedef unsigned char      uint8_t;
typedef uint16_t           u16;
typedef uint8_t            u8;
typedef char               int8;
typedef unsigned char      uint8;
typedef unsigned int       uint32;
typedef int                int32;
typedef uint16_t           uint16;

#ifndef NULL
#define NULL 0
#endif

#define RET_SUCCESS     (0)
#define RET_FAILED      (-1)

#define OK_STR     "OK"
#define NO_STR     "NO"

#define LOCKCLIENTNUM_MAX     20 /* 网关最多挂20个锁 */
#define LOCKCLIENTSTATUS_MAX  1 /* 每个锁最多存最近10个状态 */
typedef uint8_t (*eventCb)( void *arg );

/* 保存配置参数 */
__packed typedef struct _configData
{
    uint32_t magicnum;
    uint8_t LockNum;
    /*配置过的锁ID*/
    uint8_t Lock_Id[LOCKCLIENTNUM_MAX][12];/* +1 for printf */
    uint8_t Volume;
} configdata_t;
__packed typedef struct _lockData
{
    uint8_t flag;  /* flag=1代表数据还未处理过 */
    uint8_t sn;
    uint8_t Lock_Status;
    uint8_t Lock_Alert;
    uint8_t Lock_Power;
    uint32_t timeStamp; /* 时间戳 */
} lockData_t;
/* 锁客户端数据结构体 */
__packed typedef struct _lockClient
{
    uint8_t *Lock_Id;
    lockData_t LockData[LOCKCLIENTSTATUS_MAX];
}lockClient_t;


/* 锁的全局变量结构体 */
__packed typedef struct _SMARTLOCK_T
{
    uint16_t        status;
    // uint8_t         wifiStatus;
    configdata_t    pConfigData;
    lockClient_t    LockClient[LOCKCLIENTNUM_MAX];
} smartlock_t;

#endif