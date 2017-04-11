#include "comm.h"
#include "cc1101.h"


static uint8 _rssiValue=0;
static uint8 _lqiValue= 0;
static ccEnv_T cc1101Env;
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
    memset( cc1101Env,0,sizeof( cc1101Env) );
    cc1101Log( INFO,"cc1101Env size =%d\n",sizeof(cc1101Env) );

    cc1101_WriteData( CC1101_REG_IOCFG1,IOCFG1 );
    cc1101_WriteData( CC1101_REG_FIFOTHR,FIFOTHR );
    cc1101_WriteData( CC1101_REG_PKTLEN,PKTLEN );
    cc1101_WriteData( CC1101_REG_PKTCTRL1,PKTCTRL1 );
    cc1101_WriteData( CC1101_REG_PKTCTRL0,PKTCTRL0 );
    cc1101_WriteData( CC1101_REG_ADDR,ADDR );
    cc1101_WriteData( CC1101_REG_CHANNR,CHANNR );
    cc1101_WriteData( CC1101_REG_FSCTRL1,FSCTRL1 );
    cc1101_WriteData( CC1101_REG_FSCTRL0,FSCTRL0 );
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
    cc1101_AddrFilter( 3 );
    cc1101_AddrWrite( 0XAA );
    cc1101_GDOxCFG( 2,0x06 );
    cc1101_GDOxCFG( 0,0x06 );
    cc1101_SyncWordWrite( syncWord );
    cc1101_PackLenMode( PKTCTRL0 );
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
        if( _rssiValue>=128 )
        {
            cc1101Env.RSSI = (_rssiValue-256)/2 - 74;
        }
        else
        {
            cc1101Env.RSSI = (_rssiValue)/2 - 74;
        }
    }
    return cc1101Env.RSSI;
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
        cc1101Env.LQI = _lqiValue;
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
    cc1101Env.syncWord[0] = pSyncWord[0];
    cc1101Env.syncWord[1] = pSyncWord[1];
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
            cc1101_WriteData( CC1101_REG_IOCFG0,value );
            break;
        case 1:
            cc1101_WriteData( CC1101_REG_IOCFG1,value );
            break;
        case 2:
            cc1101_WriteData( CC1101_REG_IOCFG2,value );
            break;
        default :
            return RET_FAILED;
            break;
    }
    return RET_SUCCESS;
}

/******************************************************************
Function    :   cc1101_AddrFilter
说明        :   cc1101 地址过滤设置
flag        :   0x00:无地址检测 0x01: 地址检测，不接收广播
                0x02:地址检测，且接收广播地址为0x00
                0x03:地址检查，且接收广播地址为0x00 和0xFF
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-02
******************************************************************/
int8 cc1101_AddrFilter( uint8 flag )
{
    uint8 bit[2]=0,i=0;
    uint8 value=0;
    if( !(0==flag || 0x01==flag || 0x02==flag || 0x03==flag) )
    {
        assert();
        return RET_FAILED;
    }
    value = cc1101_ReadData( CC1101_REG_PKTCTRL1 );
    cc1101Log( INFO,"%s Before Write value :%d \n",__FUNCTION__, value );
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
    cc1101Env.addrFilterMode = flag;
    cc1101_WriteData(  CC1101_REG_PKTCTRL1,value );
    cc1101Log( INFO,"bit0=%d bit1=%d \n",bit[0],bit[1] );
    cc1101Log( INFO,"%s After Write value :%d \n",__FUNCTION__,value );
    return RET_SUCCESS;
}


/******************************************************************
Function    :   cc1101_AddrWrite
说明        :   cc1101 地址设置
flag        :   APP_DISABLE|APP_ENABLE
value       :   地址内容
return      :   RET_SUCCESS/RET_FAILED
Add by AlexLin    --2017-04-11
******************************************************************/
void cc1101_AddrWrite( uint8 addr )
{
    cc1101_WriteData( CC1101_REG_ADDR,addr );
    cc1101Env.masterAddr = addr;
    cc1101Log( INFO,"%s addr=0x%02x\n",__FUNCTION__,addr );
    return ;
}

/******************************************************************
Function    :   cc1101_AddrRead
说明        :   cc1101 读取
return      :   cc1101 地址值
Add by AlexLin    --2017-04-11
******************************************************************/
uint8 cc1101_AddrRead( void )
{
    return cc1101_ReadData( CC1101_REG_ADDR );
}

/******************************************************************
Function    :   cc1101_PackLenConfig
说明        :   cc1101 数据包长度格式设置
flag        :   0x00:固定数据包长度，由PKTLEN指定发送长度
                0x01:可变长度数据包，通过同步词汇后的第一个位配置数据包长度
                0x10:启用无限长度数据包
return      :   rssi data
Add by AlexLin    --2017-04-01
******************************************************************/
void cc1101_PackLenMode( uint8 flag )
{
    uint8 value = 0;
    value  = get_bit(flag,1)*2+get_bit( flag,0 );
    if( !(0x00==value || 0x01==value || 0x10==value)  )
    {
        assert();
    }
    else
    {
        cc1101Env.packLenMode = value;
        cc1101Log( INFO,"%s value=%d \n",__FUNCTION__, value );
    }

}

/******************************************************************
Function    :   cc1101_Receive
说明        :   cc1101 接收数据
pData       :   存放数据的指针
dataLen     :   pData,指针指向的缓存大小
return      :   rssi data
Add by AlexLin    --2017-04-01
******************************************************************/
int16 cc1101_Rece( uint8 *pData,uint8 dataLen )
{
    static uint8 cc1101Time=0;
    uint8 len=0,i=0;
    uint8 rssi=0,lqi=0;
    int8 ack_flag=0;
    uint8 payloadLen=0;
    cc1101Log( DUMP,"Sync world and wait the end of packet...\n",);

    while( GDO_2_READ );
    len = cc1101_ReadData( CC1101_REG_RXBYTES );
    cc1101Log( DUMP,"%s dev data len=%d\n",__FUNCTION__,len );
    if( len<=0 )
    {
        cc1101_ModeSet(IDLE_Mode);
        cc1101_ModeSet( Rx_Mode );
        return RET_FAILED;
    }
    RF_CS_LOW;
    SPI2_SendByte(BURST_READ_FIFO);
    if( dataLen < len )
    {
        for(i = 0;i < len;i ++)
        {
            SPI2_SendByte(0xff);
        }
        cc1101Log( ERROR,"%s dataLen=%d is less than the dev data len=%d\n",__FUNCTION__,dataLen,len );

        RF_CS_HIGH;
        cc1101_ModeSet(IDLE_Mode);
        cc1101_ModeSet( Rx_Mode );
        return RET_FAILED;
    }
    if( 0x01==cc1101Env.packLenMode )
    {
        payloadLen = SPI2_SendByte(0xff);
        cc1101Log( INFO,"payload len :%02x\n",payloadLen );
        len--;
    }
    if( 0x00!=cc1101Env.addrFilterMode )
    {
         cc1101Env.clientAddr = SPI2_SendByte(0xff);
         cc1101Log( INFO,"clientAddr:%02x\n",cc1101Env.clientAddr );
         len--;
    }
    if( get_bit(PKTCTRL1,2) )
    {
        len-=2;
    }
    for(i = 0;i < len;i ++)
    {

        pData[i] = SPI2_SendByte(0xff);

        logDump( " %02X",pData[i] );
    }
    logDump("\r\n");
    if( get_bit(PKTCTRL1,2) )
    {
        uint8 lastDatap[2];
        lastDatap[0] = SPI2_SendByte(0xff);
        lastDatap[1] = SPI2_SendByte(0xff);
        _rssiValue  = lastDatap[0];
        cc1101Env.LQI = _lqiValue = (lastDatap[1]&0x7F);
        if( _rssiValue>=128 )
        {
            cc1101Log( INFO,"%s %d rssi=%d\n",__FUNCTION__,__LINE__,_rssiValue );
            cc1101Env.RSSI = (char)((_rssiValue-256)/2 - 74);
        }
        else
        {

            cc1101Env.RSSI= ((_rssiValue)/2 - 74);
            cc1101Log( INFO,"%s %d rssi=%d %+d\n",__FUNCTION__,__LINE__,_rssiValue,cc1101Env.RSSI );
        }

        ack_flag = get_bit( lastDatap[1],7 );
    }
    cc1101Time++;
    RF_CS_HIGH;


    cc1101_ModeSet(IDLE_Mode);
    cc1101_ModeSet( Rx_Mode );

    cc1101Log( INFO,"rssi=%d dBm LQI=%02x ack=%d time=%d\n\n\n",cc1101Env.RSSI,_lqiValue,ack_flag,cc1101Time );
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