#include "timer.h"
#include "pwm.h"
#include "common.h"
#include "protect.h"
/*****************************************************************************
函 数 名  	       : Timer0_Init
功能描述  	: TIMER0初始化
输入参数  	: null
输出参数  	: null
返 回 值 		: null
*****************************************************************************/

void Timer0_Init()
{
    SFRPAGE = 0x00;
    TPSC = 0x00;
#if (TIMERFREQUENCY == 8)
    TPSC |= BIT0; // 8分频
#elif (TIMERFREQUENCY == 64)
    TPSC |= BIT1; // 64分频
#elif (TIMERFREQUENCY == 256)
    TPSC |= BIT0;
    TPSC |= BIT1; // 256分频
#endif
    TMOD = 0x01; // 16BIT控制
    TL0 = 0x00;  // 低8位
    TH0 = 0x00;  // 高8位
}

void Timer2_Init()
{
    SFRPAGE = 0x00;
    T2CON |= BIT0;  // 16bit定时器
    T2PSC &= BIT0F; //清除第0位
    T2PSC &= BIT1F; //清除第1位
#if (TIMERFREQUENCY == 8)
    T2PSC |= BIT0; // 8分频
#elif (TIMERFREQUENCY == 64)
    TPSC |= BIT3; // 64分频
#elif (TIMERFREQUENCY == 256)
    TPSC |= BIT2;
    TPSC |= BIT3; // 256分频
#endif
    TL2 = 0x00;
    TH2 = 0X00;
    TIMER2_STOP;
}
/**************************************************************
 函 数 名  : Timer2_ISR
 功能描述  : 延迟换相30°
 输入参数  : 无
 输出参数  : void
**************************************************************/
void Timer2_ISR() interrupt 8
{
    TIMER2_RESET;
    if (Halless.zero_flag)
    {
        Halless.zero_flag = 0;
        HoldParm.Stall_Cnt = 0;
        PWMSwitchPhase();
    }
    else
    {
        if (++HoldParm.Stall_Cnt >= 10)
        {
            TIMER2_STOP;
            Fault_ErrorCommutation();
            HoldParm.Stall_Cnt = 0;
        }
    }
}