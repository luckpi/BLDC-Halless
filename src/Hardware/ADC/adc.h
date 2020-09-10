#ifndef _ADC_H
#define _ADC_H
#include "typedef.h"
// 通道号
#define CHL_VBUS (0x04)
#define CHL_IBUS (0x0b)
#define CHL_UBemf (0x08)
#define CHL_VBemf (0x00)
#define CHL_WBemf (0x03)
#define IBUS_GAIN 1 // 0 1 2 4 8 16 32 64 定位及长短波电流采样放大倍数

// INP-PA1 IBUS+	INN-PA2 IBUS-
#if (IBUS_GAIN == 0) // 1/2倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x18; \
    }
#elif (IBUS_GAIN == 1) // 1倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x19; \
    }
#elif (IBUS_GAIN == 2) // 2倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1a; \
    }
#elif (IBUS_GAIN == 4) // 4倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1b; \
    }
#elif (IBUS_GAIN == 8) // 8倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1c; \
    }
#elif (IBUS_GAIN == 16) // 16倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1d; \
    }
#elif (IBUS_GAIN == 32) // 32倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1e; \
    }
#elif (IBUS_GAIN == 64) // 64倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x1f; \
    }
#else // 1倍放大
#define CMP_PGA_I       \
    {                   \
        SFRPAGE = 0x00; \
        ADCPGAC = 0x19; \
    }
#endif
extern void ADC_Init();
extern void ADCExtTrigEnable();
extern u16 GetVoltageValue();
extern u16 Get_CHL_Value();
extern void Start_ADCSample(u8 ADC_CHL);
extern void Switch_ADC_CHL(u8 ADC_CHL);
#endif