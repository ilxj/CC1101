#ifndef CC1101_H_
#define CC1101_H_
#include "comm.h"
#include "hal_spi.h"

#define RF_CS_HIGH          GPIO_SetBits(RF_GPIO, RF_SC_PIN)
#define RF_CS_LOW           GPIO_ResetBits(RF_GPIO, RF_SC_PIN)

#define RF_SCK_HIGH         GPIO_SetBits(RF_GPIO, SPI2_SCK)
#define RF_SCK_LOW          GPIO_ResetBits(RF_GPIO, SPI2_SCK)

#define RF_MISO_HIGH        GPIO_SetBits(RF_GPIO, SPI2_MISO)
#define RF_MISO_LOW         GPIO_ResetBits(RF_GPIO, SPI2_MISO)
#define RF_MISO_READ        GPIO_ReadInputDataBit(RF_GPIO, SPI2_MISO)

#define RF_MOSI_HIGH        GPIO_SetBits(RF_GPIO, SPI2_MOSI)
#define RF_MOSI_LOW         GPIO_ResetBits(RF_GPIO, SPI2_MOSI)
#define RF_MOSI_READ        GPIO_ReadInputDataBit(RF_GPIO, SPI2_MOSI)

#define GDO_0_HIGH          GPIO_SetBits(RF_GPIO, GDO_0)
#define GDO_0_LOW           GPIO_ResetBits(RF_GPIO, GDO_0)
#define GDO_0_READ          GPIO_ReadInputDataBit(RF_GPIO, GDO_0)

#define GDO_2_HIGH          GPIO_SetBits(RF_GPIO, GDO_2)
#define GDO_2_LOW           GPIO_ResetBits(RF_GPIO, GDO_2)
#define GDO_2_READ          GPIO_ReadInputDataBit(RF_GPIO, GDO_2)

/*CC1101命令掩码  */
#define WRITE_SINGLE        0x00                        //单独写
#define WRITE_BURST         0x40                        //连续写入
#define READ_SINGLE         0x80                        //单独读
#define READ_BURST          0xC0                        //连续读
#define BYTES_IN_RXFIFO     0x7F                        //接收缓冲区的有效字节数
#define CRC_OK              0x80                        //CRC校验通过位标志

/*FIFO命令掩码*/
#define BURST_READ_FIFO     0xff        //突发读取RX FIFO
#define BYTE_READ_FIFO      0xBF        //单字节读取 RX FIFO
#define BURST_WRITE_FIFO    0x7f        //突发写TX FIFO
#define BYTE_WRITE_FIFO     0x3f        //单字节写 TX FIFO

//CC1101寄存器定义
typedef enum
{
    //可读写的寄存器
    CC1101_REG_IOCFG2       =   0x00,   //GDO2输出脚配置
    CC1101_REG_IOCFG1       =   0x01,   //GDO1输出脚配置
    CC1101_REG_IOCFG0       =   0x02,   //GDO0输出脚配置
    CC1101_REG_FIFOTHR      =   0x03,   //RX FIFO和TX FIFO门限
    CC1101_REG_SYNC1        =   0x04,   //同步词汇，高字节
    CC1101_REG_SYNC0        =   0x05,   //同步词汇，低字节
    CC1101_REG_PKTLEN       =   0x06,   //数据包长度
    CC1101_REG_PKTCTRL1     =   0x07,   //数据包自动控制
    CC1101_REG_PKTCTRL0     =   0x08,   //数据包自动控制
    CC1101_REG_ADDR         =   0x09,   //设备地址
    CC1101_REG_CHANNR       =   0x0a,   //信道数
    CC1101_REG_FSCTRL1      =   0x0b,   //频率合成器控制，高字节
    CC1101_REG_FSCTRL0      =   0x0c,   //频率合成器控制，低字节
    CC1101_REG_FREQ2        =   0x0d,   //频率控制词汇，高字节
    CC1101_REG_FREQ1        =   0x0e,   //频率控制词汇，中间字节
    CC1101_REG_FREQ0        =   0x0f,   //频率控制词汇，低字节
    CC1101_REG_MDMCFG4      =   0x10,   //调制器配置
    CC1101_REG_MDMCFG3      =   0x11,   //调制器配置
    CC1101_REG_MDMCFG2      =   0x12,   //调制器配置
    CC1101_REG_MDMCFG1      =   0x13,   //调制器配置
    CC1101_REG_MDMCFG0      =   0x14,   //调制器配置
    CC1101_REG_DEVIATN      =   0x15,   //调制器背离设置
    CC1101_REG_MCSM2        =   0x16,   //主通信控制状态机配置
    CC1101_REG_MCSM1        =   0x17,   //主通信控制状态机配置
    CC1101_REG_MCSM0        =   0x18,   //主通信控制状态机配置
    CC1101_REG_FOCCFG       =   0x19,   //频率偏移补偿配置
    CC1101_REG_BSCFG        =   0x1a,   //位同步配置
    CC1101_REG_AGCTRL2      =   0x1b,   //AGC控制
    CC1101_REG_AGCTRL1      =   0x1c,   //AGC控制
    CC1101_REG_AGCTRL0      =   0x1d,   //AGC控制
    CC1101_REG_WOREVT1      =   0x1e,   //高字节时间0暂停
    CC1101_REG_WOREVT0      =   0x1f,   //低字节时间0暂停
    CC1101_REG_WORCTRL      =   0x20,   //电磁波激活控制
    CC1101_REG_FREND1       =   0x21,   //前末端RX配置
    CC1101_REG_FREND0       =   0x22,   //前末端TX配置
    CC1101_REG_FSCAL3       =   0x23,   //频率合成器校准
    CC1101_REG_FSCAL2       =   0x24,   //频率合成器校准
    CC1101_REG_FSCAL1       =   0x25,   //频率合成器校准
    CC1101_REG_FSCAL0       =   0x26,   //频率合成器校准
    CC1101_REG_RCCTRL1      =   0x27,   //RC振荡器配置
    CC1101_REG_RCCTRL0      =   0x28,   //RC振荡器配置
    CC1101_REG_FSTEST       =   0x29,   //频率合成器校准控制
    CC1101_REG_PTEST        =   0x2a,   //产品测试
    CC1101_REG_AGCTEST      =   0x2b,   //AGC测试
    CC1101_REG_TEST2        =   0x2c,   //不同的测试设置
    CC1101_REG_TEST1        =   0x2d,   //不同的测试设置
    CC1101_REG_TEST0        =   0x2e,   //不同的测试设置
    //只读的状态寄存器,如果写入将导致命令滤波
    CC1101_REG_PARTNUM      =   0xf0,   //CC2550的组成部分数目
    CC1101_REG_VERSION      =   0xf1,   //当前版本数
    CC1101_REG_FREQEST      =   0xf2,   //频率偏移估计
    CC1101_REG_LQI          =   0xf3,   //连接质量的解调器估计
    CC1101_REG_RSSI         =   0xf4,   //接收信号强度指示
    CC1101_REG_MARCSTATE    =   0xf5,   //控制状态机状态
    CC1101_REG_WORTIME1     =   0xf6,   //WOR计时器高字节
    CC1101_REG_WORTIME0     =   0xf7,   //WOR计时器低字节
    CC1101_REG_PKTSTATUS    =   0xf8,   //当前GDOx状态和数据包状态
    CC1101_REG_VCOVCDAC     =   0xf9,   //PLL校准模块的当前设定
    CC1101_REG_TXBYTES      =   0xfA,   //TX FIFO中的下溢和比特数
    CC1101_REG_RXBYTES      =   0xfB,   //RX FIFO中的下溢和比特数
    //手册上面没有的
    CC1101_REG_STATUS1      =   0xfc,   //
    CC1101_REG_STATUS0      =   0xfd,   //
    //功率控制
    CC1101_REG_PATABLE0     =   0x40,
    CC1101_REG_PATABLE1     =   0x41,
    CC1101_REG_PATABLE2     =   0x42,
    CC1101_REG_PATABLE3     =   0x43,
    CC1101_REG_PATABLE4     =   0x44,
    CC1101_REG_PATABLE5     =   0x45,
    CC1101_REG_PATABLE6     =   0x46,
    CC1101_REG_PATABLE7     =   0x47,
}CC1101_REG_TYPE;
enum CC1101_CmdAddr
{
    // Strobe commands
    CCxxx0_SRES      =   0x30,        // Reset chip.
    CCxxx0_SFSTXON   =   0x31,        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
    CCxxx0_SXOFF     =   0x32,        // Turn off crystal oscillator.
    CCxxx0_SCAL      =   0x33,        // Calibrate frequency synthesizer and turn it off// (enables quick start).
    CCxxx0_SRX       =   0x34,        // Enable RX. Perform calibration first if coming from IDLE and// MCSM0.FS_AUTOCAL=1.
    CCxxx0_STX       =   0x35,        // In IDLE state: Enable TX. Perform calibration first if// MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
    // Only go to TX if channel is clear.
    CCxxx0_SIDLE     =   0x36,        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
    CCxxx0_SAFC      =   0x37,        // Perform AFC adjustment of the frequency synthesizer
    CCxxx0_SWOR      =   0x38,        // Start automatic RX polling sequence (Wake-on-Radio)
    CCxxx0_SPWD      =   0x39,        // Enter power down mode when CSn goes high.
    CCxxx0_SFRX      =   0x3A,        // Flush the RX FIFO buffer.
    CCxxx0_SFTX      =   0x3B,        // Flush the TX FIFO buffer.
    CCxxx0_SWORRST   =   0x3C,        // Reset real time clock.
    CCxxx0_SNOP      =   0x3D        // No operation. May be used to pad strobe commands to two
};

//enum CC1101_STATUS_REG
//{
    //只读的状态寄存器,如果写入将导致命令滤波
//    CC1101_REG_PARTNUM      =   0xf0,   //CC2550的组成部分数目
//    CC1101_REG_VERSION      =   0xf1,   //当前版本数
//    CC1101_REG_FREQEST      =   0xf2,   //频率偏移估计
//    CC1101_REG_LQI          =   0xf3,   //连接质量的解调器估计
//    CC1101_REG_RSSI         =   0xf4,   //接收信号强度指示
//    CC1101_REG_MARCSTATE    =   0xf5,   //控制状态机状态
//    CC1101_REG_WORTIME1     =   0xf6,   //WOR计时器高字节
//    CC1101_REG_WORTIME0     =   0xf7,   //WOR计时器低字节
//    CC1101_REG_PKTSTATUS    =   0xf8,   //当前GDOx状态和数据包状态
//    CC1101_REG_VCOVCDAC     =   0xf9,   //PLL校准模块的当前设定
//    CC1101_REG_TXBYTES      =   0xfA,   //TX FIFO中的下溢和比特数
//    CC1101_REG_RXBYTES      =   0xfB,   //RX FIFO中的下溢和比特数
//};
enum CC1101_Mode
{
    Rx_Mode=0,
    Tx_Mode,
    IDLE_Mode
};
typedef enum  _cc1101_env
{
    CC_RSSI=0,
    CC_LQI,
    CC_SyncWord,
}CC1101_ENV;
typedef struct _cc1101
{
  int32 data;
}CC1101_T;


uint8 cc1101_RSSI( uint8 flag );
uint8 cc1101_LQI( uint8 flag );
void cc1101_Init();
void cc1101_ModeSet( enum CC1101_Mode mode );
uint8 cc1101_Rece( uint8 *pData );
#endif