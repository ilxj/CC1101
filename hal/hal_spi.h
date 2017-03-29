#ifndef HAL_SPI_H_
#define HAL_SPI_H_
#include "stm32f0xx.h"

#define RF_GPIO     GPIOB
#define GDO_0       GPIO_Pin_10   //SI4432有数据此引脚拉低,要设置成中断
#define GDO_2       GPIO_Pin_11   //高电平 SI4432所有数据丢失
#define RF_SC_PIN   GPIO_Pin_12   //低电平芯片被选中
#define SPI2_SCK    GPIO_Pin_13
#define SPI2_MISO   GPIO_Pin_14
#define SPI2_MOSI   GPIO_Pin_15

void hal_RFGPIO_Init();
void hal_spiInit();
uint8_t SPI2_SendByte( uint8_t byte );
#endif