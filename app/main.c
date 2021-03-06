#include "comm.h"
#include "cc1101.h"
#include "core_cmFunc.h"
void buildInfo()
{
  logDump("\r\n\r\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");
  log( INFO,"Build Time:%s %s\n",__DATE__,__TIME__);
}
uint32 lastTime=0;
int main( void )
{
  uint8 buf[20];
  SystemInit();
  SysTick_Config( SystemCoreClock / SycTi );
  uartOpen(1,115200);
  uartOpen(2,9600);
  buildInfo();
  Delay_Init( 48 );
  cc1101_Init();

  WatchDog_Init( 5*1000 );
  TIM2_Int_Init( 4799,9 );
  cc1101_ModeSet( IDLE_Mode );
  cc1101_ModeSet( Rx_Mode );
  log( INFO,"bufLen=%d \n",sizeof(buf)/sizeof(buf[0]) );
  while(1)
  {
      if( getTime_S()-lastTime >=10 )
      {
         lastTime = getTime_S();
         buf[11] = lastTime;
         //cc1101_Send( buf,0X0D );
      }
      if( GDO_2_READ )
      {
        cc1101_Rece( buf,20 );
      }
      WatchDog_Feed();
  }
  return 0;
}
