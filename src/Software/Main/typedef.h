#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#include "jmt18f003.h"

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;
typedef signed char s8;
typedef signed int s16;
typedef signed long s32;

#define BIT0 (0x01) // (0<<1)		0000 0001
#define BIT1 (0x02) // (1<<1)
#define BIT2 (0x04) // (2<<1)
#define BIT3 (0x08) // (3<<1)
#define BIT4 (0x10) // (4<<1)
#define BIT5 (0x20) // (5<<1)
#define BIT6 (0x40) // (6<<1)
#define BIT7 (0x80) // (7<<1)

#define BIT0F (0xfe) // (0<<1)  	1111 1110
#define BIT1F (0xfd) // (1<<1)
#define BIT2F (0xfb) // (2<<1)
#define BIT3F (0xf7) // (3<<1)
#define BIT4F (0xef) // (4<<1)
#define BIT5F (0xdf) // (5<<1)
#define BIT6F (0xbf) // (6<<1)
#define BIT7F (0x7f) // (7<<1)

#define NONE 0

#define MQ15(Float_Value) ((Float_Value < 0.0) ? (s16)(32768 * (Float_Value)-0.5) : (s16)(32767 * (Float_Value) + 0.5))

#define ALL_INT_EN \
    {              \
        EA = 1;    \
    }

#define ALL_INT_DISEN \
    {                 \
        EA = 0;       \
    }

#define CMP_INT_EN    \
    {                 \
        IEN2 |= BIT0; \
    }

#define CMP_INT_DISEN  \
    {                  \
        IEN2 &= BIT0F; \
    }

#define EXINT_INT_EN  \
    {                 \
        IEN0 |= BIT0; \
    } // 1：允许 EXINT 中断

#define EXINT_INT_DISABLE \
    {                     \
        IEN0 &= BIT0F;    \
    } // 0：禁止 EXINT 中断
#define ADC_INT_EN    \
    {                 \
        IEN1 |= BIT4; \
    }

#define ADC_INT_DISEN  \
    {                  \
        IEN1 &= BIT4F; \
    }

#define TIMER0_INT_EN \
    {                 \
        IEN0 |= BIT1; \
    }

#define TIMER0_INT_DISEN \
    {                    \
        IEN0 &= BIT1F;   \
    }

#define TIMER0_RESET    \
    {                   \
        SFRPAGE = 0x00; \
        TH0 = 0x00;     \
        TL0 = 0x00;     \
    }

#define TIMER0_START    \
    {                   \
        SFRPAGE = 0x00; \
        TR0 = 1;        \
    }

#define TIMER0_STOP     \
    {                   \
        SFRPAGE = 0x00; \
        TR0 = 0;        \
    }

#define TIMER2_START    \
    {                   \
        SFRPAGE = 0x00; \
        T2CON |= BIT4;  \
    }

#define TIMER2_STOP     \
    {                   \
        SFRPAGE = 0x00; \
        T2CON &= BIT4F; \
    }

#define TIMER2_RESET    \
    {                   \
        SFRPAGE = 0x00; \
        TH2 = 0x00;     \
        TL2 = 0x00;     \
        RH2 = 0x00;     \
        RL2 = 0x00;     \
    }

#define TIMER1_INT_EN \
    {                 \
        IEN0 |= BIT3; \
    }

#define TIMER1_INT_DISEN \
    {                    \
        IEN0 &= BIT3F;   \
    }

/*全部中断关闭*/
#define TOTAL_INT_DISEN \
    {                   \
        IEN0 = 0x00;    \
        IEN1 = 0x00;    \
        IEN2 = 0x00;    \
    }

#define TIMER2_INT_EN \
    {                 \
        IEN1 |= BIT3; \
    }

#define TIMER2_INT_DISEN \
    {                    \
        IEN1 &= BIT3F;   \
    }

#define PWM1_INT_EN   \
    {                 \
        IEN1 |= BIT2; \
    }

#define PWM1_INT_DISEN \
    {                  \
        IEN1 &= BIT2F; \
    }

#define PWM1_START         \
    {                      \
        SFRPAGE = 0x01;    \
        TIM1_CNTEN = 0x01; \
    }

#define PWM1_STOP          \
    {                      \
        SFRPAGE = 0x01;    \
        TIM1_CNTEN = 0x00; \
    }

#define SOFT_RESET        \
    {                     \
        SFRPAGE = 0x02;   \
        GLOBALRST = 0x01; \
    } // 全局软复位
#define ADC_IF_CLEAR      \
    {                     \
        SFRPAGE = 0x00;   \
        ADCCON0 &= BIT6F; \
    } // ADC清除中断和通道状态

#define ADC_CHL_CLEAR    \
    {                    \
        SFRPAGE = 0x00;  \
        ADCCON0 &= 0xf0; \
    } // ADC中断
#define EXINT_IR_Num interrupt 0
#define TIMER0_IR_Num interrupt 1
#define TIMER1_IR_Num interrupt 3
#define UART_IR_Num interrupt 4
#define WDT_IR_Num interrupt  5
#define PWM0_IR_Num interrupt 6
#define PWM1_IR_Num interrupt 7
#define TIMER2_IR_Num interrupt 8
#define ADC_IR_Num interrupt 9
#define LIN_IR_Num interrupt 12
#define CMP_IR_Num interrupt 14
#define SPI_IR_Num interrupt 17
#define I2C_IR_Num interrupt 19
#endif
