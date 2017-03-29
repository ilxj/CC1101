#ifndef TIMER_H_
#define TIMER_H_
#include "comm.h"
#include "typedef.h"
void dumpSysClocks( void );
void TIM3_Int_Init(u16 arr,u16 psc);
#endif