#ifndef __UART_H__
#define __UART_H__

#include "jmt18f003.h"
#include "typedef.h"

#define EN_UART_TX_ADC 0 // 中断打印

#define EN_UART_STR_Ps 1  // 打印信息
#define EN_UART_STR_Pss 0 // 打印版本

#if (EN_UART_STR_Ps)
#define Ps(x, y) UART_SEND_STR(x, y);
#else
#define Ps(x, y)
#endif
#if (EN_UART_STR_Pss)
#define Pss(x, y) UART_SEND_STR(x, y);
#else
#define Pss(x, y)
#endif

extern void UART_Init();

#if (EN_UART_TX_ADC)
extern void UART_TX_EXT(u16 u16data);
#endif

#if (EN_UART_STR_Ps || EN_UART_STR_Pss)
extern void UART_TX(u8 u8data);
extern void UART_SEND_STR(char *myStr, u16 myNumber);
#endif

#endif
