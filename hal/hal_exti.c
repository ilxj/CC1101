#include "comm.h"
#include "hal_exti.h"
//struct list_head tempMsgHead;

void EXTI0_1_IRQHandler(void){ 
    if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
    {  
        //退出中断时注意清除标志位  
        EXTI_ClearFlag(EXTI_Line0);  
        Delay_ms(10);
        if(GPIO_ReadInputDataBit( GPIOA, GPIO_Pin_0 ))
            return;
        log(INFO,"Key press PA0 \n");
    }  
    if(EXTI_GetITStatus(EXTI_Line1)!=RESET)  
    {  
        //退出中断时注意清除标志位  
        EXTI_ClearFlag(EXTI_Line1);  
        Delay_ms(10);
        if(GPIO_ReadInputDataBit( GPIOA, GPIO_Pin_1 ))
            return;
        log(INFO,"Key press PA1 \n");
    }  
}  
void EXTI4_15_IRQHandler( void )
{
    if(EXTI_GetITStatus(EXTI_Line10)!=RESET)  
    {  
        //退出中断时注意清除标志位  
        EXTI_ClearFlag(EXTI_Line10);  
        Delay_ms(10);
        if(GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_10 ))
            return;
    }  
    if( EXTI_GetITStatus(EXTI_Line6)!=RESET )
    {
        EXTI_ClearFlag(EXTI_Line6);
        Delay_ms(10);
        if(GPIO_ReadInputDataBit( GPIOF, GPIO_Pin_6 ))
        return;
        log( INFO,"%s %d SMARTLOCK_SI4432REBOOT\n",__FUNCTION__,__LINE__ );
    }
    if( EXTI_GetITStatus(EXTI_Line7)!=RESET )
    {
        EXTI_ClearFlag(EXTI_Line7);
        log( INFO,"sound play Over!\n",__FUNCTION__,__LINE__ );
    }
}
void keyInit()
{
    //时钟使能  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOF,ENABLE);  
    /* SYSCFG Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);    

    //外部按键GPIO初始华，PA0  PA1
    GPIO_InitTypeDef GPIO_InitStructure;  
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
    GPIO_Init(GPIOA,&GPIO_InitStructure);  
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;  
    GPIO_Init(GPIOF,&GPIO_InitStructure); 

    //将EXTI0指向PA0  
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0); 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);  
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF,EXTI_PinSource6);  
    //EXTI0中断线配置  
    EXTI_InitTypeDef EXTI_InitStructure;  
    EXTI_InitStructure.EXTI_Line=EXTI_Line0|EXTI_Line1|EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  

    //EXTI0中断向量配置  
    NVIC_InitTypeDef NVIC_InitStructure;  
    NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;  
    // NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_Init(&NVIC_InitStructure); 
    log(INFO,"InIt KEY1-PA0  KEY2-PA1 KEY3-PF6 OK.\n");
}


void WatchDog_Init( uint32_t ms )
{
    int timeout_ms=0;
    timeout_ms = (ms/6.4);
    if(timeout_ms>= 0xfff)
       timeout_ms = 0xffe;
    log( INFO,"%s %d.\n",__FUNCTION__,__LINE__);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);
    IWDG_SetReload(timeout_ms);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void WatchDog_Feed( void )
{
  IWDG_ReloadCounter();
}
