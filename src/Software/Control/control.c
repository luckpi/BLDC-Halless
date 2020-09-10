#include "control.h"
#include "pwm.h"
#include "init.h"
#include "halless.h"
#include "adc.h"
#include "protect.h"
#include "cmp.h"
#include "timer.h"
#include "uart.h"
#include "debug.h"
#include "ipd.h"
#include "pid.h"
/**************************************************************************************************
 函 数 名  : MotorAhead
 功能描述  : 开始启动
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
static void MotorAhead()
{
    Ps("\nStart", NONE);
    PWMPortShut();
    Fault_InitOverUnderVoltage(); // 过压保护
    mcState = mcInit;
}
/**************************************************************************************************
 函 数 名  : MotorInit
 功能描述  : 初始化
 输入参数  : 无
 输出参数  : void
**************************************************************************************************/
static void MotorInit()
{
    PWMPortShut();
    Common_Init();
    TOTAL_INT_DISEN;
    ALL_INT_DISEN;
    PWMPortBrake(); //充电
    Delay_ms(10);
    PWMPortShut();
    Delay_us(50);
    mcState = mcAlign;
}

/*****************************************************************************
 函 数 名  : MotorAlign
 功能描述  : 
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorAlign()
{
    HoldParm.PWMDutyCycle = PWM_START_DUTY;
    PWMChangeDuty(HoldParm.PWMDutyCycle);
    // PWMOutput();
    // PWMSwitchPhase();
    // Delay_ms(100);
    // PWMPortShut();
    // Delay_us(50);
    IPD();
    // Halless.HallessState = IPD(); //获取初始相位
    // halless_to_phase();           //相位转换
    PWMOutput();
    SFRPAGE = 0x02; // 中断标志清零
    EXINTCON = 0x00;
    ALL_INT_EN;
    ADCExtTrigEnable();
    CMP_INT_EN;
    TIMER2_INT_EN;
    mcState = mcDrag;
}

/*****************************************************************************
 函 数 名  : StartupDrag
 功能描述  : 启动
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void StartupDrag()
{
    static u16 ADC_CNT = 0;
    static u8 CNT = 0;
    CheckZeroCrossing();
    ADC_CNT++;
    if (Halless.zero_flag)
    {
        Halless.zero_flag = 0; //此处用作标识位
        if (++CNT > 18)
        {
            CNT = 20;
            Halless.Filter_Cnt = 5;
            EnterRunInit();
            return;
        }
        PWMSwitchPhase();
        ADC_CNT = 0;
    }
    else if (ADC_CNT >= HoldParm.DragTime)
    {
        ADC_CNT = 0;
        CNT = 0;
        HoldParm.DragTime -= ((HoldParm.DragTime / 30) + 1);
        if (HoldParm.DragTime < 300)
        {
            HoldParm.DragTime = 300;
        }
        if (++Halless.Phase > 5)
            Halless.Phase = 0;
        PWMSwitchPhase();
        Halless.Filter_Count = 0;
    }
    HoldParm.PWMDutyCycle += 1;
    if (HoldParm.PWMDutyCycle < PWM_START_DUTY) // 限制输出占空比的最大最小值
    {
        HoldParm.PWMDutyCycle = PWM_START_DUTY;
    }
    else if (HoldParm.PWMDutyCycle > PWM_DUTYCYCLE_20)
    {
        HoldParm.PWMDutyCycle = PWM_DUTYCYCLE_20;
    }
    PWMChangeDuty(HoldParm.PWMDutyCycle);
}
/*****************************************************************************
 函 数 名  : EnterRunInit
 功能描述  : 进入Run的初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void EnterRunInit()
{
    if (++Halless.Phase > 5)
        Halless.Phase = 0;
    PWMSwitchPhase();
    TIMER0_START;
    TIMER2_START;
    mcState = mcRun;
}
/*****************************************************************************
 函 数 名  : MotorRun
 功能描述  : 运行
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorRun()
{
    if (HoldParm.SpeedLoopCnt > 100)
    {
        HoldParm.SpeedLoopCnt = 0;
        pi_spd.Fdb = HoldParm.RPM;
        PID_CLAC(&pi_spd);
        HoldParm.PWMDutyCycle += pi_spd.Out;
        UP16LIMIT(HoldParm.PWMDutyCycle, PWM_DUTYCYCLE_75, PWM_DUTYCYCLE_15);
        PWMChangeDuty(HoldParm.PWMDutyCycle);
        Ps("RPM=", HoldParm.RPM);
    }
}

/*****************************************************************************
 函 数 名  : MotorStop
 功能描述  : 电机停止转动
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorStop()
{
    PWMPortShut();
    ALL_INT_DISEN;
    TIMER0_STOP;
    TIMER2_STOP;
    TIMER0_RESET;
    TIMER2_RESET;
    // Ps("Stop\n", NONE);
}

/*****************************************************************************
 函 数 名  : MotorFault
 功能描述  : 电机等待重启状态，电流保护，CMP保护，堵转保护。
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
static void MotorFault()
{
    Debug(); // 打印错误信息
    Ps("\nloading reboot\n", NONE);
}

/*****************************************************************************
 函 数 名  : MotorControl
 功能描述  : 电机控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorControl()
{
    switch (mcState)
    {
    case mcAhead:
        MotorAhead();
        break;
    case mcInit:
        MotorInit();
        break;
    case mcAlign:
        MotorAlign();
        break;
    case mcDrag:
        break;
    case mcRun:
        MotorRun();
        break;
    case mcStop:
        MotorStop();
        break;
    case mcFault:
        MotorFault();
        break;
    default:
        MotorStop();
        break;
    }
}
