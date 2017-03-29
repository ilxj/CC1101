#include "comm.h"
#include "hal_spi.h"

uint8_t SPI2_SendByte(uint8_t byte)
{
    uint8_t ret =0;
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
        SPI_SendData8(SPI2,byte);
    // SPI_I2S_SendData16(SPI2, byte);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);//是否已经读取
    /* Return the byte read from the SPI bus */
    ret = SPI_ReceiveData8(SPI2);
    // ret = SPI_I2S_ReceiveData16(SPI2);
    return ret;//SPI 接收
}

void hal_RFGPIO_Init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB,ENABLE);

    GPIO_InitStruct.GPIO_Pin = GDO_0|GDO_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;//GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;
    GPIO_Init(RF_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = RF_SC_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RF_GPIO,&GPIO_InitStruct);
}
void hal_spiInit()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);

    GPIO_PinAFConfig(RF_GPIO, GPIO_PinSource5,GPIO_AF_0);
    GPIO_PinAFConfig(RF_GPIO, GPIO_PinSource6,GPIO_AF_0);
    GPIO_PinAFConfig(RF_GPIO, GPIO_PinSource7,GPIO_AF_0);

    GPIO_InitStruct.GPIO_Pin = SPI2_SCK|SPI2_MISO|SPI2_MOSI;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(RF_GPIO, &GPIO_InitStruct);

    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;

    SPI_Init(SPI2, &SPI_InitStruct);
    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
    SPI_Cmd(SPI2, ENABLE);
    log(INFO,"spi init ok!\n");
}
u8 SPI_ExchangeByte( u8 input )
{
    /* Loop while DR register in not emplty */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
  /* Send byte through the SPI1 peripheral */
  SPI_SendData8(SPI2, input);
  /* Wait to receive a byte */
  while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
  /* Return the byte read from the SPI bus */
  return SPI_ReceiveData8(SPI2);
}
