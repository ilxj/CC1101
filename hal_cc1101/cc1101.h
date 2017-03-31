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

enum CC1101_Addr
{
    IOCFG2 = 0x0000,
    IOCFG1,
    IOCFG0,
    FIFOTHR,
    SYNC1,
    SYNC0,
    PKTLEN,
    PKTCTRL1,
    PKTCTRL0,
    ADDR,
    CHANNR,
    FSCTRL1,
    FSCTRL0,
    FREQ2,
    FREQ1,
    FREQ0,
    MDMCFG4,
    MDMCFG3,
    MDMCFG2,
    MDMCFG1,
    MDMCFG0,
    DEVIATN,
    MCSM2,
    MCSM1,
    MCSM0,
    FOCCFG,
    BSCFG,
    AGCCTRL2,
    AGCCTRL1,
    AGCCTRL0,
    WOREVT1,
    WOREVT0,
    WORCTRL,
    FREND1,
    FREND0,
    FSCAL3,
    FSCAL2,
    FSCAL1,
    FSCAL0,
    RCCTRL1,
    RCCTRL0,
    FSTEST,
    PTEST,
    AGCTEST,
    TEST2,
    TEST1,
    TEST0 = 0x002E,
    //read only reg
    PARTNUM = 0x0030,
    VERSION,
    FREQEST,
    LQI,
    RSSI,
    MARCSTATE,
    WORTIME1,
    WORTIME0,
    PKTSTATUS,
    VCO_VC_DAC,
    TXBYTES,
    RXBYTES,
    RCCTRL1_STATUS,
    RCCTRL0_STATUS,
};
enum CC1101_CmdAddr
{
    // Strobe commands
    CCxxx0_SRES      =   0x30,        // Reset chip.
    CCxxx0_SFSTXON   =   0x31,        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
    CCxxx0_SXOFF     =   0x32,        // Turn off crystal oscillator.
    CCxxx0_SCAL      =   0x33,        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
    CCxxx0_SRX       =   0x34,        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
    CCxxx0_STX       =   0x35,        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
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

enum CC1101_STATUS_REG
{
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
};
enum CC1101_Mode
{
    Rx_Mode=0,
    Tx_Mode,
    IDLE_Mode
};
typedef struct _cc1101
{
  int32 data;
}CC1101_T;



void cc1101_Init();
void cc1101_ModeSet( enum CC1101_Mode mode );
uint8 cc1101_Receive( uint8 *pData );
#endif