#include "halless.h"
#include "common.h"
#include "pwm.h"
#include "debug.h"
#include "uart.h"
#include "adc.h"
#include "control.h"
/*****************************************************************************
 函 数 名  : CalcAverageSpeedTime
 功能描述  : 对转速求平均
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void CalcAvgSpeedTime()
{
    HoldParm.SpeedTime_Sum += HoldParm.SpeedTimeTemp;

    if (++HoldParm.SpeedTime_Cnt >= 8)
    {
        HoldParm.SpeedTime_Cnt = 0;
        HoldParm.SpeedTime = HoldParm.SpeedTime_Sum >> 3;
        HoldParm.SpeedTime_Sum = 0;
    }
    if (HoldParm.SpeedTime)
    {
        HoldParm.RPM = SPEEDFACTOR / HoldParm.SpeedTime;
    }
    else
    {
        HoldParm.RPM = SPEEDFACTOR / HoldParm.SpeedTimeTemp;
    }
}

static void CalcSpeedTime()
{
    SFRPAGE = 0x00;
    HoldParm.SpeedTimeTemp = (TH0 << 8) + TL0;
    TIMER0_RESET;
    if (Halless.Filter_Count <= 12 && mcState == mcRun) //堵转检测
    {
        if (++HoldParm.MainDetectCnt >= 50)
        {
            PWMPortShut();
        }
    }
    else
    {
        HoldParm.MainDetectCnt = 0;
    }
    Halless.delay_time = 65535 - (HoldParm.SpeedTimeTemp >> 3);
    TH2 = Halless.delay_time >> 8;
    TL2 = Halless.delay_time & 0xff;
    // TIMER2_START;
    Halless.Filter_Count = 0;
    Halless.zero_flag = 1;
    // PWMSwitchPhase();
    CalcAvgSpeedTime();
}
void CheckZeroCrossing()
{
    if (++Halless.Filter_Count >= Halless.Filter_Cnt)
    {
        // Halless.Filter_Count = Halless.Filter_Cnt;
        ADCSample.NeutralPoint = (ADCSample.UBemf + ADCSample.VBemf + ADCSample.WBemf) / 3;
        if (HoldParm.RotorDirection == CW)
        {
            switch (Halless.Phase)
            {
            case 0:
                if (ADCSample.WBemf < ADCSample.NeutralPoint) //6
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 1:
                if (ADCSample.VBemf > ADCSample.NeutralPoint) //2
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 2:
                if (ADCSample.UBemf < ADCSample.NeutralPoint) //3
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 3:
                if (ADCSample.WBemf > ADCSample.NeutralPoint) //1
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 4:
                if (ADCSample.VBemf < ADCSample.NeutralPoint) //5
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 5:
                if (ADCSample.UBemf > ADCSample.NeutralPoint) //4
                {
                    Halless.Phase = 0;
                    CalcSpeedTime();
                }
                break;
            default:
                Halless.Phase = 0;
                break;
            }
        }
        else if (HoldParm.RotorDirection == CCW)
        {
            switch (Halless.Phase)
            {
            case 0:
                if (ADCSample.WBemf > ADCSample.NeutralPoint) //1
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 1:
                if (ADCSample.UBemf < ADCSample.NeutralPoint) //3
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 2:
                if (ADCSample.VBemf > ADCSample.NeutralPoint) //2
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 3:
                if (ADCSample.WBemf < ADCSample.NeutralPoint) //6
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 4:
                if (ADCSample.UBemf > ADCSample.NeutralPoint) //4
                {
                    Halless.Phase++;
                    CalcSpeedTime();
                }
                break;
            case 5:
                if (ADCSample.VBemf < ADCSample.NeutralPoint) //5
                {
                    Halless.Phase = 0;
                    CalcSpeedTime();
                }
                break;
            default:
                Halless.Phase = 0;
                break;
            }
        }
    }
}

/*****************************************************************************
 函 数 名  : ThreeBemfSample
 功能描述  : 三相BEMF采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ThreeBemfSample()
{
    if (ADCSample.ChlState == CHL_UBemf)
    {
        ADCSample.UBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_VBemf;
    }
    else if (ADCSample.ChlState == CHL_VBemf)
    {
        ADCSample.VBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_WBemf;
    }
    else if (ADCSample.ChlState == CHL_WBemf)
    {
        ADCSample.WBemf = Get_CHL_Value();
        ADCSample.ChlState = CHL_UBemf;
    }
}
/*****************************************************************************
 函 数 名  : ADCAnalogSample
 功能描述  : 采样，电流、电压
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void ADCAnalogSample()
{
    Start_ADCSample(CHL_IBUS);
    ADCSample.Current = Get_CHL_Value();
    ADCSample.Sum += ADCSample.Current;
    ADCSample.Num++;
    if (ADCSample.Num == 8)
    {
        ADCSample.Average = (ADCSample.Sum >> 3);
        ADCSample.Sum = 0;
        ADCSample.Num = 0;
    }
    Start_ADCSample(CHL_VBUS);
    ADCSample.Voltage = Get_CHL_Value();
    Switch_ADC_CHL(ADCSample.ChlState);
}
/*****************************************************************************
 函 数 名  : ADC_ISR
 功能描述  : ADC中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADC_ISR() ADC_IR_Num
{
    volatile u16 u16Temp = 0;
    ADC_IF_CLEAR;
    switch (mcState)
    {
    case mcDrag:
        ThreeBemfSample();
        ADCAnalogSample();
        StartupDrag();
        break;
    case mcRun:
        HoldParm.SpeedLoopCnt++;
        ThreeBemfSample();
        ADCAnalogSample();
        CheckZeroCrossing();
        break;
    default:
        break;
    }
#if (EN_UART_TX_ADC)
//u16Temp = mcState;
//u16Temp = HoldParm.PWMDutyCycle;
//u16Temp = HoldParm.RequireSpeed;
//u16Temp = ADCSample.Current;
u16Temp = ADCSample.Voltage;
//u16Temp = ADCSample.CurrentOffset;
// u16Temp = ADCSample.Average;
//u16Temp = ADCSample.Temperature;

//u16Temp = HoldParm.RequireSpeed;

//u16Temp = HoldParm.RotorDirection;

//u16Temp = HallParm.HallState;
//u16Temp = HallParm.HallUniteState;
//u16Temp = HallParm.HallTime;
UART_TX_EXT(u16Temp);
#endif
}