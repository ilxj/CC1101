#ifndef DELAY_H_
#define DELAY_H_ 

void Delay_Init( uint8_t SYSCLK );
void Delay_us(uint32_t nus);
void Delay_ms(uint16_t nms);
#endif