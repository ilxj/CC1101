#ifndef _HAL_UART_H
#define _HAL_UART_H

/* USART Communication boards Interface */
// UART 1
#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
#define USARTx_IRQn                      USART1_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_1

#define USARTx_RX_PIN                    GPIO_Pin_10          
#define USARTx_RX_GPIO_PORT              GPIOA              
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_1

void uartOpen( char port ,uint32_t bps );
uint16_t uart1Send( uint8_t *data, uint16_t length );
uint16_t uart2Send( uint8_t *data, uint16_t length );

#endif