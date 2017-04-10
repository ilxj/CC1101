#include "comm.h"
#include "cc1101.h"

static CC1101_T cc1101;
static uint8 _rssiValue=0;
static uint8 _lqiValue= 0;
// Rf settings for CC1101
static char rfSettings[] = {
    0x06,  // IOCFG2              GDO2 Output Pin Configuration
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
    uint8 ret=0;

    RF_CS_LOW;
    while ( RF_MISO_READ );
    ret = SPI2_SendByte( cmd );
    RF_CS_HIGH;
    return ret;
}

/******************************************************************
Function    :   cc1101_WriteData
说明        :   向CC1101对应地址写入数据
addr        :   要写入数据的地址值
value       :   需要写入的数据
return      ：  CC1101的底层状态
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

/******************************************************************
Function    :   cc1101_ReadData
说明        :   向CC1101对应地址读取数据
addr        :   要读数据的地址值
return      ：  读出的数据
Add by AlexLin    --2017-04-05
******************************************************************/
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
    uint8 syncWord[2]={ SYNC0,SYNC1 };

    cc1101_WriteData( CC1101_REG_IOCFG1,IOCFG1 );
    cc1101_WriteData( CC1101_REG_PKTLEN,PKTLEN );
    cc1101_WriteData( CC1101_REG_PKTCTRL1,PKTCTRL1 );
    cc1101_WriteData( CC1101_REG_PKTCTRL0,PKTCTRL0 );
    cc1101_WriteData( CC1101_REG_ADDR,ADDR );
    cc1101_WriteData( CC1101_REG_CHANNR,CHANNR );
    cc1101_WriteData( CC1101_REG_IOCFG1,FSCTRL1 );
    cc1101_WriteData( CC1101_REG_FSCTRL1,FSCTRL0 );
    cc1101_WriteData( CC1101_REG_FREQ2,FREQ2 );
    cc1101_WriteData( CC1101_REG_FREQ1,FREQ1 );
    cc1101_WriteData( CC1101_REG_FREQ0,FREQ0 );
    cc1101_WriteData( CC1101_REG_MDMCFG4,MDMCFG4 );
    cc1101_WriteData( CC1101_REG_MDMCFG3,MDMCFG3 );
    cc1101_WriteData( CC1101_REG_MDMCFG2,MDMCFG2 );
    cc1101_WriteData( CC1101_REG_MDMCFG1,MDMCFG1 );
    cc1101_WriteData( CC1101_REG_MDMCFG0,MDMCFG0 );
    cc1101_WriteData( CC1101_REG_DEVIATN,DEVIATN );
    cc1101_WriteData( CC1101_REG_MCSM2,MCSM2 );
    cc1101_WriteData( CC1101_REG_MCSM1,MCSM1 );
    cc1101_WriteData( CC1101_REG_MCSM0,MCSM0 );
    cc1101_WriteData( CC1101_REG_FOCCFG,FOCCFG );
    cc1101_WriteData( CC1101_REG_BSCFG,BSCFG );
    cc1101_WriteData( CC1101_REG_AGCCTRL2,AGCCTRL2 );
    cc1101_WriteData( CC1101_REG_AGCCTRL1,AGCCTRL1 );
    cc1101_WriteData( CC1101_REG_AGCCTRL0,AGCCTRL0 );
    cc1101_WriteData( CC1101_REG_WOREVT1,WOREVT1 );
    cc1101_WriteData( CC1101_REG_WOREVT0,WOREVT0 );
    cc1101_WriteData( CC1101_REG_WORCTRL,WORCTRL );
    cc1101_WriteData( CC1101_REG_FREND1,FREND1 );
    cc1101_WriteData( CC1101_REG_FREND0,FREND0 );
    cc1101_WriteData( CC1101_REG_FSCAL3,FSCAL3 );
    cc1101_WriteData( CC1101_REG_FSCAL2,FSCAL2 );
    cc1101_WriteData( CC1101_REG_FSCAL1,FSCAL1 );
    cc1101_WriteData( CC1101_REG_FSCAL0,FSCAL0 );
    cc1101_WriteData( CC1101_REG_RCCTRL1,RCCTRL1 );
    cc1101_WriteData( CC1101_REG_RCCTRL0,RCCTRL0 );
    cc1101_WriteData( CC1101_REG_FSTEST,FSTEST );
    cc1101_WriteData( CC1101_REG_PTEST,PTEST );
    cc1101_WriteData( CC1101_REG_AGCTEST,AGCTEST );
    cc1101_WriteData( CC1101_REG_TEST2,TEST2 );
    cc1101_WriteData( CC1101_REG_TEST1,TEST1 );
    cc1101_WriteData( CC1101_REG_TEST0,TEST0 );
    cc1101_GDOxCFG( 2,0x06 );
    cc1101_GDOxCFG( 0,0x06 );
    cc1101_SyncWordWrite( syncWord );
    cc1101_AddrFilterEnable( 1 );
    for( i=0;i<=0x2e;i++ )
    {
        status =  cc1101_ReadData(i);
        cc1101Log( INFO,"%02x value=%02x \n",i,status );
    }

    cc1101Log( CRITICAL,"%s ... [%s]\n",__FUNCTION__,OK_STR );
}

/******************************************************************
Function    :   cc1101_PowerReset
说明        :   cc1101 上电复位,单位dBm
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
    SPI2_SendByte( CCxxx0_SRES );
    while( RF_MISO_READ );
    RF_CS_HIGH;
    cc1101Log( CRITICAL,"%s ... [%s]\n",__FUNCTION__,OK_STR );
}
/******************************************************************
Function    :   cc1101_RSSI
说明        :   cc1101 RSSI 获取
flag        :   0XFF,更新RSSI,
return      ：  rssi data
Add by AlexLin    --2017-03-29
******************************************************************/
uint8 cc1101_RSSI( uint8 flag )
{
    if( 0xFF==flag )
    {
        _rssiValue = cc1101_ReadData( CC1101_REG_RSSI );
    }
    return _rssiValue;
}


/******************************************************************
Function    :   cc1101_LQI
说明        :   cc1101 LQI 获取
flag        :   0XFF,更新LQI.
return      :   rssi data
Add by AlexLin    --2017-04-01
******************************************************************/
uint8 cc1101_LQI( uint8 flag )
{
    if( 0XFF==flag )
    {
        _lqiValue = cc1101_ReadData( CC1101_REG_LQI );
    }
    return _lqiValue;
}

/******************************************************************
Function    :   cc1101_ModeSet
说明        :   cc1101 模式切换
mode        :   Rx_Mode|Tx_Mode|IDLE_Mode
Rx_Mode     :   接收模式
Tx_Mode     :   发送模式
IDLE_Mode   :   Idle模式
return      :   NULL
Add by AlexLin    --2017-04-01
******************************************************************/
void cc1101_ModeSet( enum CC1101_Mode mode )
{
    switch( mode )
    {
        case Rx_Mode:
            cc1101Log( INFO,"%s : Rx_Mode\n",__FUNCTION__ );
            cc1101_WriteCmd( CCxxx0_SRX );
            break;
        case Tx_Mode:
            cc1101Log( INFO,"%s : Tx_Mode\n",__FUNCTION__ );
            cc1101_WriteCmd( CCxxx0_STX );
            break;
        case IDLE_Mode:
            cc1101Log( INFO,"%s : IDLE_Mode\n",__FUNCTION__ );
            cc1101_WriteCmd( CCxxx0_SIDLE );
            break;
        default:
            cc1101Log( INFO,"%s : InValid=%d\n",__FUNCTION__,mode );
            break;
    }
}

/******************************************************************
Function    :   cc1101_SyncWordWrite
说明        :   cc1101 设置同步字，2个字节大小
pSyncWord   :   同步字指针,指向4个字节内容的同步字
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-01
******************************************************************/
int8 cc1101_SyncWordWrite( uint8 *pSyncWord )
{
    cc1101Log( INFO,"%s SyncWord:%02x %02x\n",__FUNCTION__, pSyncWord[0],pSyncWord[1] );
    cc1101_WriteData( CC1101_REG_SYNC0,pSyncWord[0] );
    cc1101_WriteData( CC1101_REG_SYNC1,pSyncWord[1] );
    return RET_SUCCESS;
}

/******************************************************************
Function    :   cc1101_SyncWordRead
说明        :   cc1101 读取同步字，2个字节大小
pSyncWord   :   同步字指针,指向4个字节内容的同步字
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-02
******************************************************************/
int8 cc1101_SyncWordRead( uint8 *pSyncWord )
{
    pSyncWord[0] = cc1101_ReadData( CC1101_REG_SYNC0 );
    pSyncWord[1] = cc1101_ReadData( CC1101_REG_SYNC1 );
    return RET_SUCCESS;
}


/******************************************************************
Function    :   cc1101_GDOxCFG
说明        :   cc1101 GDOx 引脚配置
GDOX_NUM    :   0|1|2
value       :   要配置的值
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-02
******************************************************************/
int8 cc1101_GDOxCFG( uint8 GDOX_NUM,uint8 value )
{
    cc1101Log( INFO,"%s GDOx=%d value=%02x\n",__FUNCTION__,GDOX_NUM,value );
    switch( GDOX_NUM )
    {
        case 0:
            cc1101_WriteData( CC1101_REG_IOCFG2,value );
            break;
        case 1:
            cc1101_WriteData( CC1101_REG_IOCFG1,value );
            break;
        case 2:
            cc1101_WriteData( CC1101_REG_IOCFG0,value );
            break;
        default :
            return RET_FAILED;
            break;
    }
    return RET_SUCCESS;
}

/******************************************************************
Function    :   cc1101_AddrFilterEnable
说明        :   cc1101 地址过滤使能/不使能
flag        :   APP_DISABLE|APP_ENABLE
value       :   要配置的值
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-02
******************************************************************/
int8 cc1101_AddrFilterEnable( uint8 flag )
{
    uint8 bit[2]=0,i=0;
    uint8 value=0;

    value = cc1101_ReadData( CC1101_REG_PKTCTRL1 );
    cc1101Log( INFO,"Before Write value :%d \n",value );
    bit[0] = get_bit( flag,0 );
    bit[1] = get_bit( flag,1 );
    for ( i = 0; i < 2; i++ )
    {
        /* code */
        if( 1==bit[i] )
        {
            set_bit( value,i );
        }
        else
        {
            reset_bit( value,i );
        }
    }
    cc1101Log( INFO,"bit0=%d bit1=%d \n",bit[0],bit[1] );
    cc1101Log( INFO,"After Write value :%d \n",value );
    return RET_FAILED;
}

void cc1101_AddrWrite( uint8 addr )
{

}


uint8 cc1101_AddrRead()
{
    return 0;
}

/******************************************************************
Function    :   cc1101_Receive
说明        :   cc1101 接收数据
flag        :   0XFF,更新LQI.
return      :   rssi data
Add by AlexLin    --2017-04-01
******************************************************************/
uint8 cc1101_Rece( uint8 *pData )
{
    static uint8 cc1101Time=0;
    uint8 len=0,i=0;
    uint8 rssi=0,lqi=0;
    int8 ack_flag=0;
    cc1101Log( INFO,"Sync world \n");
    cc1101Log( INFO,"wait the end of packet...\n");
    while( GDO_2_READ );
    len = cc1101_ReadData( CC1101_REG_RXBYTES );
    cc1101Log( INFO,"%s receiveLen=%d\n",__FUNCTION__,len );

    RF_CS_LOW;
    SPI2_SendByte(BURST_READ_FIFO);
    for(i = 0;i < len;i ++)
    {

        pData[i] = SPI2_SendByte(0xff);

        logDump( " %02X",pData[i] );
    }
    cc1101Time++;
    _rssiValue = pData[len-2];
    _lqiValue = pData[len-1]&0x7F;
    ack_flag =( (pData[len-1] &( 1<<7 )) >> 7 );
    RF_CS_HIGH;
    logDump("\r\n");

    cc1101_ModeSet(IDLE_Mode);
    cc1101_ModeSet( Rx_Mode );

    cc1101Log( INFO,"rssi=%02x LQI=%02x ack=%d time=%d\n\n\n",_rssiValue,_lqiValue,ack_flag,cc1101Time );
    return len;
}

/******************************************************************
Function    :   cc1101_Send
说明        :   cc1101 发送数据,最长的数据长度是64个字节
pData       :   需要发送的数据指针
len         :   数据长度
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-01
******************************************************************/
uint8 cc1101_Send( uint8 *pData,uint8 len )
{
    uint16 i=0;
    cc1101_ModeSet( IDLE_Mode );    //退出当前模式
    cc1101_WriteCmd( CCxxx0_SFTX );    //清空发送缓冲区
    // CC1101_WriteTxFIFO(pBuff, len);     //写入数据到发送缓冲区
    RF_CS_LOW;
    SPI2_SendByte( BURST_WRITE_FIFO );
    SPI2_SendByte( len );
    for( i = 0;i < len;i ++ )
    {
        SPI2_SendByte( pData[i] );
    }
    RF_CS_HIGH;
    cc1101_ModeSet( Tx_Mode );     //开始发送数据
    while( !GDO_2_READ );
    while( GDO_2_READ );
    cc1101_ModeSet( IDLE_Mode );    //退出当前模式
    cc1101_ModeSet( Rx_Mode );
    return RET_SUCCESS;
}

void cc1101_Init()
{
    cc1101_GPIOInit();
    cc1101_PowerReset();
    cc1101_RegInit();
}