#ifndef CC1101_H_
#define CC1101_H_
#include "comm.h"
#include "hal_spi.h"

#define RF_CS_HIGH          GPIO_SetBits(RF_GPIO, RF_SC_PIN)
#define RF_CS_LOW           GPIO_ResetBits(RF_GPIO, RF_SC_PIN)

#define RF_SCK_HIGH         GPIO_SetBits(RF_GPIO, SPI2_SCK)
#define RF_SCK_LOW          GPIO_ResetBits(RF_GPIO, SPI2_SCK)

#define RF_MISO_HIGH        GPIO_SetBits(RF_GPIO, SPI2_MISO)
#define RF_MISO_LOW         GPIO_ResetBits(RF_GPIO, SPI2_MISO)
#define RF_MISO_READ        GPIO_ReadInputDataBit(RF_GPIO, SPI2_MISO)

#define RF_MOSI_HIGH        GPIO_SetBits(RF_GPIO, SPI2_MOSI)
#define RF_MOSI_LOW         GPIO_ResetBits(RF_GPIO, SPI2_MOSI)
#define RF_MOSI_READ        GPIO_ReadInputDataBit(RF_GPIO, SPI2_MOSI)

#define GDO_0_HIGH          GPIO_SetBits(RF_GPIO, GDO_0)
#define GDO_0_LOW           GPIO_ResetBits(RF_GPIO, GDO_0)
#define GDO_0_READ          GPIO_ReadInputDataBit(RF_GPIO, GDO_0)

#define GDO_2_HIGH          GPIO_SetBits(RF_GPIO, GDO_2)
#define GDO_2_LOW           GPIO_ResetBits(RF_GPIO, GDO_2)
#define GDO_2_READ          GPIO_ReadInputDataBit(RF_GPIO, GDO_2)

typedef struct _cc1101
{
  int32 data;
}CC1101_T;



void cc1101_Init();
#endif