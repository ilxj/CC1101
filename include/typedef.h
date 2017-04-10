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

enum
{
    RET_FAILED=-1,
    RET_SUCCESS=0
};
typedef enum _ENABLE_FLAG
{
    APP_DISABLE=0,
    APP_ENABLE=1
}ENABLE_FLAG;

#define OK_STR     "OK"
#define NO_STR     "NO"

//对某一位置一
#define set_bit(m,n)        m|=(1<<n)
//对某一位置零:
#define reset_bit(m,n)      m &= ~( 1<<n )
//取某一位置:
#define get_bit( m,n )      ( (m &( 1<<n )) >> n )

typedef uint8_t (*eventCb)( void *arg );


#endif