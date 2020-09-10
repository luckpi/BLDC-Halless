#include "common.h"
#include "pwm.h"
#include "adc.h"
#include "halless.h"
#include "pid.h"
MotorState_T mcState;
ErrorState_T error_code;
volatile HoldControlPara_T xdata HoldParm;
volatile ADCSamplePara_T xdata ADCSample;
volatile PWMCatchPara_T xdata CatchParm;
volatile SensorPara_T xdata Halless;
/*****************************************************************************
 函 数 名  : PowerupParaInit
 功能描述  : 上电参数初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PowerupParaInit()
{
    SFRPAGE = 0x02;
    SPIIE = 0x00;  //保证重启后晶振频率一致
    JTAGEN = 0x00; // 使用PB4、PB5、PB6需配置该寄存器
    PWMPortShut();
    Common_Init();
    HoldParm.RotorDirection = CW;
    mcState = mcAhead;
    error_code = normal;
}
/*****************************************************************************
 函 数 名  : Common_Init
 功能描述  : 全局变量初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void Common_Init()
{
    HoldParm.MainDetectCnt = 0;
    HoldParm.SpeedLoopCnt = 0;
    HoldParm.PWMDutyCycle = 0;
    HoldParm.SpeedTime = 0;
    HoldParm.DragTime = 900;
    HoldParm.RPM = 0;
    HoldParm.Set_RPM = First_Gear;
    HoldParm.SpeedTime_Cnt = 0;
    HoldParm.SpeedTime_Sum = 0;
    HoldParm.SpeedTimeTemp = 0;
    HoldParm.Stall_Cnt = 0;
    ADCSample.Average = 0;
    ADCSample.Voltage = 0;
    ADCSample.OverCurrentCnt = 0;
    ADCSample.OverVoltageCnt = 0;
    ADCSample.UBemf = 0;        // 反电动势U相
    ADCSample.VBemf = 0;        // 反电动势V相
    ADCSample.WBemf = 0;        // 反电动势W相
    ADCSample.NeutralPoint = 0; // 反电动势中性点
    ADCSample.Sum = 0;
    ADCSample.Num = 0;
    ADCSample.ChlState = CHL_UBemf;
    CatchParm.Period = 0;
    CatchParm.Duty = 0;
    CatchParm.Flag_Cap_Valid = 0;
    CatchParm.PwmCnt = 0;
    CatchParm.DutyCycleUse = 0;
    error_code = normal;
    Halless.HallessState = 0;
    Halless.Phase = 0;
    Halless.delay_time = 0;
    Halless.zero_flag = 0;
    Halless.Filter_Count = 0;
    Halless.Filter_Cnt = 60;
    PID_init();
}