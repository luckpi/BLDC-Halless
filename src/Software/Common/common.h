#ifndef _COMMON_H
#define _COMMON_H
#include "typedef.h"
#define CW BIT0
#define CCW BIT1
#define First_Gear 1750  // 一档
#define Second_Gear 2000 // 二档
#define Third_Gear 2400  // 三档
// #define Filter_Cnt 5
// 控制顺序
typedef enum
{
    mcStop = 0, // 电机停止工作
    mcAhead,
    mcInit,
    mcAlign,
    mcDrag,
    mcRun,
    mcReset,
    mcFault, // 电机重启
} MotorState_T;

// 打印错误信息
typedef enum
{
    normal = 0,   // 运行正常
    overcurrent,  // adc过流保护
    cmp_int,      //cmp过流保护
    overvoltage,  // 过压
    undervoltage, // 欠压
    hallerror,    // 霍尔错误
    stall,        // 堵转
} ErrorState_T;

// 电机运行方向、占空比等参数
typedef struct
{
    u8 RotorDirection : 2; // 电机转动方向
    u8 MainDetectCnt : 6;  // 主循环计数
    u8 SpeedLoopCnt;       // 调整转速占空比的周期计数
    float PWMDutyCycle;    // PWM占空比
    u16 SpeedTime;         // 一个电周期时间
    u16 RPM;               // 实际转速
    u16 Set_RPM;           // 设定转速
    u16 DragTime;          // 强拖时间
    u32 SpeedTime_Sum;     // 换相时间8次和
    u8 SpeedTime_Cnt;      // 换相平均数计数
    u16 SpeedTimeTemp;     // 当前换相时间
    u8 Stall_Cnt;          // 堵住计数溢出次数
} HoldControlPara_T;

// ADC相关
typedef struct
{
    u8 ChlState;       // 通道状态
    u16 CurrentOffset; // 电流放大零点
    u16 Voltage;       // 电压采样值
    u16 Current;       // 电流采样值
    u16 UBemf;         // 反电动势U相
    u16 VBemf;         // 反电动势V相
    u16 WBemf;         // 反电动势W相
    u16 NeutralPoint;  // 反电动势中性点
    u16 Sum;           // 电流平均值和
    u8 Num;            // 电流平均值计数
    u16 Average;       // 电流平均值
    u8 OverVoltageCnt; // 过压次数
    u8 OverCurrentCnt; // 过流次数
} ADCSamplePara_T;

// PWM捕获相关
typedef struct
{
    u16 Period;      // 周期
    u16 Duty;        // 电平时间
    u16 PwmCnt;      // PWM信号滤波
    u8 DutyCycleUse; // 占空比
    u8 Flag_Cap_Valid;
} PWMCatchPara_T;

typedef struct
{
    u8 HallessState : 4;
    u8 LastHallessState : 4;
    u8 Phase : 3;
    u8 LastPhase : 3;
    u8 zero_flag : 1;
    u8 Filter_Count;
    u8 Filter_Cnt;
    u16 delay_time;
} SensorPara_T;
extern volatile SensorPara_T xdata Halless;

extern MotorState_T mcState;
extern ErrorState_T error_code;
extern volatile PWMCatchPara_T xdata CatchParm;
extern volatile HoldControlPara_T xdata HoldParm;
extern volatile ADCSamplePara_T xdata ADCSample;
extern void Common_Init();
extern void PowerupParaInit();
#endif