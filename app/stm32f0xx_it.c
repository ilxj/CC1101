/**
  ******************************************************************************
  * @file    IO_Toggle/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "hal_uart.h"
#include "comm.h"
/** @addtogroup STM32F0_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t	Delay1mS;
__IO 		uint8_t 	RxOkFlag=0;
__IO 		uint8_t 	RxBuffer[1024];
__IO 		uint16_t	ReceivCount=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/



void USARTx_IRQHandler(void)
{
//  int8_t str=0;
  /* USART in mode Receiver --------------------------------------------------*/
  if (USART_GetITStatus(USARTx, USART_IT_RXNE)  != RESET)
  {
		USART_ClearFlag(USART1, USART_IT_RXNE); //Çå³ýÖÐ¶Ï±êÖ¾ 
		printf("%c",USART_ReceiveData(USARTx));
//RxBuffer[ReceivCount]=USART_ReceiveData(USARTx);
//		ReceivCount++;
//		if(ReceivCount>=1024)	
//			{ReceivCount=0;RxOkFlag=1;}
//		else;
  }
	else;
}
//void USART2_IRQHandler( void )
//{
//  if (USART_GetITStatus(USART2, USART_IT_RXNE)  != RESET)
//  {
//    USART_ClearFlag(USART2, USART_IT_RXNE); 
//    USART_SendData(USART2,USART_ReceiveData(USART2));
//    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//  }
//}
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	Delay1mS++;
  if( Delay1mS>=100 )
  {
    Delay1mS=0;
    //log( INFO,"%s %d \n",__FUNCTION__,__LINE__ );
  }
}
uint16_t capture = 0;
// extern __IO uint16_t CCR1_Val;
// extern __IO uint16_t CCR2_Val;
// extern __IO uint16_t CCR3_Val;
// extern __IO uint16_t CCR4_Val;
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    capture++;
    if( capture>=999 )
    {/* 1s 定时 */
      capture=0;
      // log( INFO,"%s %d \n",__FUNCTION__,__LINE__ );
    }
  }
}
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    capture++;
    if( capture>=10 )
    {
      //wifi_timer_ms();
    }
    if( capture>=999 )
    {/* 1s 定时 */
      capture=0;
      //wifi_timer_s();
      // log( INFO,"%s %d \n",__FUNCTION__,__LINE__ );
    }
  }
}

void EXTI2_3_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET)  
    { 
        // log( INFO,"%s EXTI_Line2\n",__FUNCTION__ );
        //中断处理  

        EXTI_ClearITPendingBit(EXTI_Line2); 
    } 
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET)  
    { 
        // log( INFO,"%s EXTI_Line3\n",__FUNCTION__ );
        //中断处理  

        EXTI_ClearITPendingBit(EXTI_Line3); 
    } 
}
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
