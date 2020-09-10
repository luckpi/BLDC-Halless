#ifndef _INIT_H
#define _INIT_H
#include "typedef.h"
void WDT_Init();
void Crpm_Init();
void Interrupt_Init();
void Port_Init();
extern void Delay_ms(u16 u16data);
extern void Delay_us(u16 u16data);
#endif