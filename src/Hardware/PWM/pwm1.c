#include "pwm1.h"
#include "uart.h"
#include "common.h"
#include "control.h"
/*****************************************************************************
函 数 名  	: PWM1_Cap_Init
功能描述  	: PWM1_Cap_Init 初始化
输入参数  	: null
输出参数  	: null
返 回 值 		: null
*****************************************************************************/
void PWM1_Cap_Init()
{
    SFRPAGE = 0x01;

    TIM1_CONR0 = BIT2;         // PWM0 _CH0、 PWM0 _CH1 和PWM0_CH2 管脚上的信号异或后连接到通道 CC0
    TIM1_TGICR0 = BIT6 | BIT1; // 复位模式，通 道 CC0 的 边 沿 检 测 信 号 为滤波后的信号
    TIM1_IER0 = BIT3;          // 允许CC2捕获中断
    TIM1_IER1 = BIT0;          // 上溢产生更新中断
    TIM1_CC0MR = BIT0;         //CC0设置捕获
    TIM1_CC1MR = BIT1;         //CC1设置捕获
    TIM1_CCENR = BIT0 | BIT2;  // CC0，CC1开启
    TIM1_CCPS = BIT2;          // CC0极性取反,即CC0上升沿捕获且复位，CC1上升沿
    TIM1_PSCL = 0x01;
    TIM1_ARRH = 0xff; // 这里是频率，自己写一个合适的值，实在不行就 0XFF
    TIM1_ARRL = 0xff;
    TIM1_CCPS = BIT0; // CC0极性取反,即CC0下降沿捕获且复位，CC1上升沿
    TIM1_RCR = 0x00;
    TIM1_BRKC = 0x00;
    TIM1_EGR = 0x00;
    TIM1_ISR0 = 0x00;
    TIM1_ISR1 = 0x00;
    PWM1_START;
}
/*****************************************************************************
 函 数 名  : CatchDutyCycle
 功能描述  : 得到外部信号占空比
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CatchDutyCycle()
{
    static bit Flag_Overflow;
    SFRPAGE = 0x01;
    if ((TIM1_ISR0 & BIT3) == BIT3) // CC0捕获中断
    {
        TIM1_ISR0 &= BIT3F;
        if (CatchParm.PwmCnt >= 3)
        {
            if (Flag_Overflow == 1) //有过上溢更新事件
            {
                Flag_Overflow = 0;
            }
            CatchParm.Period = (TIM1_CC0RH << 8) + TIM1_CC0RL;
            CatchParm.Duty = (TIM1_CC1RH << 8) + TIM1_CC1RL;
            if (CatchParm.Duty <= CatchParm.Period)
            {
                CatchParm.DutyCycleUse = 100 - (u32)CatchParm.Duty * 100 / CatchParm.Period;
                if (CatchParm.DutyCycleUse < 15 && CatchParm.DutyCycleUse > 5)
                    HoldParm.Set_RPM = First_Gear;
                else if (CatchParm.DutyCycleUse > 45 && CatchParm.DutyCycleUse < 55)
                    HoldParm.Set_RPM = Second_Gear;
                else if (CatchParm.DutyCycleUse > 85)
                    HoldParm.Set_RPM = Third_Gear;
                else if (CatchParm.DutyCycleUse < 5)
                    mcState = mcStop;
                if ((mcState == mcStop) && (CatchParm.DutyCycleUse > 5))
                {
                    mcState = mcAhead;
                }
            }
        }
        else
        {
            CatchParm.PwmCnt++;
        }
    }
    if ((TIM1_ISR1 & BIT4) == BIT4) // 上溢更新事件
    {
        TIM1_ISR1 &= BIT4F;
        Flag_Overflow = 1;
        CatchParm.PwmCnt = 0;
        if (!PA3) //0%关机
        {
            mcState = mcStop;
        }
        else //满占空比全速
        {
            HoldParm.Set_RPM = 2450;
            if (mcState == mcStop)
            {
                mcState = mcAhead;
            }
        }
    }
}
