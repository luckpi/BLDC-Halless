#include "protect.h"
#include "adc.h"
#include "control.h"
#include "uart.h"
#include "pwm.h"
#include "common.h"
#include "debug.h"
/********************************************************************************************************
 函 数 名  : Fault_OverVoltage
 功能描述  : 过压保护函数
 输入参数  : 无
 输出参数  : void
********************************************************************************************************/
void Fault_InitOverUnderVoltage()
{
    u16 VoltageValue = 0;
    while (1)
    {
        VoltageValue = GetVoltageValue();
        if (VoltageValue < OVER_VOLTAGE_PROTECT && VoltageValue > UNDER_VOLTAGE_PROTECT)
        {
            if (++ADCSample.OverVoltageCnt >= 64)
            {
                ADCSample.OverVoltageCnt = 0;
                break;
            }
        }
        else
        {
            if (VoltageValue > OVER_VOLTAGE_PROTECT)
                error_code = overvoltage;
            else
                error_code = undervoltage;
            ADCSample.OverVoltageCnt = 0;
            Debug();
        }
    }
}
/********************************************************************************************************
 函 数 名  : Protect_Voltage
 功能描述  : 过压保护函数
 输入参数  : 无
 输出参数  : void
********************************************************************************************************/
void Protect_Voltage()
{
    if (ADCSample.Voltage > (OVER_VOLTAGE_PROTECT + 20) || ADCSample.Voltage < (UNDER_VOLTAGE_PROTECT - 20))
    {
        if (++ADCSample.OverVoltageCnt >= 64)
        {
            MotorStop();
            mcState = mcAhead;
        }
    }
    else
    {
        ADCSample.OverVoltageCnt = 0;
    }
}

void Protect_Current()
{
    if (ADCSample.Average > 50)
    {

        if (++ADCSample.OverCurrentCnt > 30)
        {
            MotorStop(); //电机停止
            mcState = mcFault;
            error_code = overcurrent;
            ADCSample.OverVoltageCnt = 0;
        }
    }
    else
    {
        ADCSample.OverCurrentCnt = 0;
    }
}
/********************************************************************************************************
 函 数 名  : Fault_ErrorCommutation
 功能描述  : 堵转保护函数
 输入参数  : 无
 输出参数  : void
********************************************************************************************************/
void Fault_ErrorCommutation()
{
    MotorStop();
    mcState = mcFault;
    error_code = stall;
}