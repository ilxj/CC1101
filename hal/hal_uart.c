#include "stm32f0xx.h"
#include "comm.h"

#include <stdio.h>
#ifdef __GNUC__
// With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar()
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


PUTCHAR_PROTOTYPE
{
  USART_SendData(USART1,(uint8_t)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void USART2_IRQHandler(void)
{
   uint8_t value = 0;
  if (USART_GetITStatus(USART2, USART_IT_RXNE)  != RESET)
  {
    USART_ClearFlag(USART2, USART_IT_RXNE);
    value = USART_ReceiveData(USART2);
  }
}

static uint32_t Send_Byte (uint8_t c)
{
    USART_SendData(USART2, c);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
  return 0;
}
void SendPacket(uint8_t *data, uint16_t length)
{
  uint16_t i;
  i = 0;
  while (i < length)
  {
    Send_Byte(data[i]);
    i++;
  }
}
void uartOpen( char port ,uint32_t bps )
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

  USART_InitStructure.USART_BaudRate = bps;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  if( 1==port )
  {
    /* Enable USART clock */
    USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);
    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

    GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
    GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
    GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
    USART_DeInit(USART1);
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1,ENABLE);
    USART_ClearFlag(USART1, USART_FLAG_TC);
  }
  else
  {
    /* Enable USART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    /* Connect PXx to USARTx_Tx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, USARTx_TX_AF);
    /* Connect PXx to USARTx_Rx */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, USARTx_RX_AF);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    USART_DeInit(USART2);
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2,ENABLE);
    USART_ClearFlag(USART2, USART_FLAG_TC);
  }
  // log( INFO,"Open UART%d bps=%d\n",port,bps );
}
uint16_t uart1Send( uint8_t *data, uint16_t length )
{
  uint16_t i;
  i = 0;
  if( NULL==data )
  return -1;
  while (i < length)
  {
    USART_SendData(USART1, data[i]);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    i++;
  }
  return length;
}
uint16_t uart2Send( uint8_t *data, uint16_t length )
{
  uint16_t i,dataLen=length;

  while (i < length)
  {
    USART_SendData(USART2, data[i]);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    if( i>=2 && data[i] == 0xFF )
    {
      USART_SendData(USART2, 0x55 );
      while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    }
    i++;
  }
  return dataLen;
}

int8_t uart_write(uint8_t *buf, uint32_t len)
{
  return uart2Send( buf, len );
}



