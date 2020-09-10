#ifndef _PID_H
#define _PID_H
#include "common.h"
#include "typedef.h"
#define Speed_Kp 0.03f
#define Speed_Ki 0
#define Speed_Kd 0
typedef struct
{
	float Ref;	   // 输入: 期望输入
	float Fdb;	   // 输入: 反馈输入
	float Err;	   // 变量: 目标误差
	float L_Err;   // 历史:  上一次比例输出
	float Err_sum; // 积分误差和
	float Kp;	   // 参数: 比例增益
	float Ki;	   // 参数: 积分增益
	float Kd;	   // 参数: 微分增益
	float Up;	   // 变量: 比例输出
	float Ui;	   // 变量: 积分输出
	float Ud;	   // 变量: 微分输出
	float UiMAX;   // Err_sum限幅
	float OutMax;  // 参数: 上限 输出
	float OutMin;  // 参数: 下限 输出
	float Out;	   // 输出: PID 输出
} PIDREG_T, *p_PIDREG_T;
#define UP16LIMIT(var, max, min)               \
	{                                          \
		(var) = (var) > (max) ? (max) : (var); \
		(var) = (var) < (min) ? (min) : (var); \
	}
extern PIDREG_T xdata pi_spd;
extern void PID_init(void);
extern void PID_CLAC(PIDREG_T *PID);
#endif
