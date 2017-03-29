#include "comm.h"
#include "cc1101.h"

static CC1101_T cc1101;
// Rf settings for CC1101
static char rfSettings[] = {
    0x29,  // IOCFG2              GDO2 Output Pin Configuration
    0x2E,  // IOCFG1              GDO1 Output Pin Configuration
    0x06,  // IOCFG0              GDO0 Output Pin Configuration
    0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
    0xD3,  // SYNC1               Sync Word, High Byte
    0x91,  // SYNC0               Sync Word, Low Byte
    0xFF,  // PKTLEN              Packet Length
    0x04,  // PKTCTRL1            Packet Automation Control
    0x05,  // PKTCTRL0            Packet Automation Control
    0x00,  // ADDR                Device Address
    0x00,  // CHANNR              Channel Number
    0x06,  // FSCTRL1             Frequency Synthesizer Control
    0x00,  // FSCTRL0             Frequency Synthesizer Control
    0x10,  // FREQ2               Frequency Control Word, High Byte
    0xB1,  // FREQ1               Frequency Control Word, Middle Byte
    0x3B,  // FREQ0               Frequency Control Word, Low Byte
    0xF6,  // MDMCFG4             Modem Configuration
    0x83,  // MDMCFG3             Modem Configuration
    0x13,  // MDMCFG2             Modem Configuration
    0x22,  // MDMCFG1             Modem Configuration
    0xF8,  // MDMCFG0             Modem Configuration
    0x15,  // DEVIATN             Modem Deviation Setting
    0x07,  // MCSM2               Main Radio Control State Machine Configuration
    0x30,  // MCSM1               Main Radio Control State Machine Configuration
    0x18,  // MCSM0               Main Radio Control State Machine Configuration
    0x16,  // FOCCFG              Frequency Offset Compensation Configuration
    0x6C,  // BSCFG               Bit Synchronization Configuration
    0x03,  // AGCCTRL2            AGC Control
    0x40,  // AGCCTRL1            AGC Control
    0x91,  // AGCCTRL0            AGC Control
    0x87,  // WOREVT1             High Byte Event0 Timeout
    0x6B,  // WOREVT0             Low Byte Event0 Timeout
    0xFB,  // WORCTRL             Wake On Radio Control
    0x56,  // FREND1              Front End RX Configuration
    0x10,  // FREND0              Front End TX Configuration
    0xE9,  // FSCAL3              Frequency Synthesizer Calibration
    0x2A,  // FSCAL2              Frequency Synthesizer Calibration
    0x00,  // FSCAL1              Frequency Synthesizer Calibration
    0x1F,  // FSCAL0              Frequency Synthesizer Calibration
    0x41,  // RCCTRL1             RC Oscillator Configuration
    0x00,  // RCCTRL0             RC Oscillator Configuration
    0x59,  // FSTEST              Frequency Synthesizer Calibration Control
    0x7F,  // PTEST               Production Test
    0x3F,  // AGCTEST             AGC Test
    0x81,  // TEST2               Various Test Settings
    0x35,  // TEST1               Various Test Settings
    0x09,  // TEST0               Various Test Settings
    // 0x00,  // PARTNUM             Chip ID
    // 0x14,  // VERSION             Chip ID
    // 0x00,  // FREQEST             Frequency Offset Estimate from Demodulator
    // 0x40,  // LQI                 Demodulator Estimate for Link Quality
    // 0x80,  // RSSI                Received Signal Strength Indication
    // 0x01,  // MARCSTATE           Main Radio Control State Machine State
    // 0x00,  // WORTIME1            High Byte of WOR Time
    // 0x00,  // WORTIME0            Low Byte of WOR Time
    // 0x00,  // PKTSTATUS           Current GDOx Status and Packet Status
    // 0x94,  // VCO_VC_DAC          Current Setting from PLL Calibration Module
    // 0x00,  // TXBYTES             Underflow and Number of Bytes
    // 0x00,  // RXBYTES             Overflow and Number of Bytes
    // 0x00,  // RCCTRL1_STATUS      Last RC Oscillator Calibration Result
    // 0x00,  // RCCTRL0_STATUS      Last RC Oscillator Calibration Result
};
static void cc1101_GPIOInit( void )
{
   hal_RFGPIO_Init();
   hal_spiInit();
   cc1101Log( CRITICAL,"%s ... [%s]\n",__FUNCTION__,OK_STR );
}

/******************************************************************
Function    :   cc1101_WriteCmd
说明        :   向CC1101发送命令
return      ：  NULL
Add by AlexLin    --2017-03-29
******************************************************************/
static uint8 cc1101_WriteCmd( uint8 cmd )
{
    return SPI2_SendByte( cmd );
}

/******************************************************************
Function    :   cc1101_WriteData
说明        :   向CC1101对应地址写入数据
addr        :   要写入数据的地址值
value       :   需要写入的数据
return      ：  NULL
Add by AlexLin    --2017-03-29
******************************************************************/
static uint8 cc1101_WriteData( uint8 addr,uint8 value )
{
    RF_CS_LOW;
    while ( RF_MISO_READ );
    SPI2_SendByte( addr);       //先写地址
    SPI2_SendByte(value);      //再写数据
    RF_CS_HIGH;
    return RET_SUCCESS;
}
static uint8 cc1101_ReadData( uint8 addr )
{
    uint8 status=0;
    RF_CS_LOW;
    while ( RF_MISO_READ );
    SPI2_SendByte( 0x80|addr );       //先写地址
    status = SPI2_SendByte( 0xff );      //再写数据
    RF_CS_HIGH;
    return status;
}
/******************************************************************
Function    :   cc1101_RegInit
说明        :   cc1101 寄存器初始化
return      ：  NULL
Add by AlexLin    --2017-03-29
******************************************************************/
static void cc1101_RegInit( void )
{
    uint8 i=0,addr=0,status=0;
    uint8 regNum = sizeof(rfSettings)/sizeof(rfSettings[0]);
    cc1101Log( CRITICAL,"%s regNum=%d\n",__FUNCTION__,regNum );
    // cc1101Log( INFO,"Write addr=%02x value=%02x\n",addr,rfSettings[i] );
    // cc1101_WriteData( 0,0X29 );
    // Delay_ms(100);
    // status = cc1101_ReadData( 0 );
    // cc1101Log( INFO,"Read addr=%02x value=%02x\n",0,status );

    for( i=0;i<regNum;i++ )
    {
        if( i>0X2E )
        {
            addr = i+1;
        }
        else
        {
            addr = i;
        }
        cc1101Log( INFO,"Write addr=%02x value=%02x\n",addr,rfSettings[i] );
        cc1101_WriteData( addr,rfSettings[i] );
        // Delay_ms(1);
    }
    for( i=0;i<regNum;i++ )
    {
        if( i>0X2E )
        {
            addr = i+1;
        }
        else
        {
            addr = i;
        }
        // Delay_ms(100);
        status = cc1101_ReadData( addr );
        cc1101Log( INFO,"Read addr=%02x value=%02x\n",addr,status );
    }

    cc1101Log( CRITICAL,"%s ... [%s]\n",__FUNCTION__,OK_STR );
}

/******************************************************************
Function    :   cc1101_PowerReset
说明        :   cc1101 上电复位
return      ：  NULL
Add by AlexLin    --2017-03-29
******************************************************************/
static void cc1101_PowerReset( void )
{
    RF_CS_HIGH;
    RF_SCK_HIGH;
    RF_MOSI_LOW;
    Delay_ms( 100 );
    RF_CS_LOW;
    Delay_ms(1);
    RF_CS_HIGH;
    Delay_ms(1);
    RF_CS_LOW;
    while( RF_MISO_READ );
    cc1101_WriteCmd( 0X30 );
    while( RF_MISO_READ );
    RF_CS_HIGH;
    cc1101Log( CRITICAL,"%s ... [%s]\n",__FUNCTION__,OK_STR );
}
void cc1101_Init()
{
    cc1101_GPIOInit();
    cc1101_PowerReset();
    cc1101_RegInit();

}