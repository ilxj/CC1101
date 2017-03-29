#include "comm.h"
#include "core_cmFunc.h"
void buildInfo()
{
  logDump("\r\n\r\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\r\n");
  log( INFO,"Build Time:%s %s\n",__DATE__,__TIME__);
}
int main( void )
{
  SystemInit();
  SysTick_Config( SystemCoreClock / SycTi );
  uartOpen(1,115200);
  uartOpen(2,9600);
  buildInfo();
  Delay_Init( 48 );
  cc1101_Init();
  //WatchDog_Init( 5*1000 );
  TIM2_Int_Init( 4799,9 );
  while(1)
  {

  }
  return 0;
}
