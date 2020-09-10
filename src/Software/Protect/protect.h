#ifndef _PROTECT_H
#define _PROTECT_H
#include "typedef.h"
#define OVER_VOLTAGE_PROTECT (900)
#define UNDER_VOLTAGE_PROTECT (700)
#define Fault_Error_CNT (40000)
extern void Protect_Voltage();
extern void Fault_InitOverUnderVoltage();
extern void Protect_Current();
extern void Fault_ErrorCommutation();
#endif