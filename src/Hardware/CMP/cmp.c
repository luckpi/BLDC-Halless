#include "cmp.h"
#include "common.h"
#include "adc.h"
#include "control.h"
#include "pwm.h"
#include "uart.h"
/*****************************************************************************
 函 数 名  : CMP_Init
 功能描述  : 比较器初始化，电流保护
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CMP_Init()
{
	CMP_INT_DISEN;
	CMP_PGA_I;
	SFRPAGE = 0x02;
	CMP_CLK_DIV = 0x0d; //比较器滤波系数
	SFRPAGE = 0x00;
	CMPEN = 0x00;  // 正常工作模式
	CMPEN |= 0x04; // bit 2-1:00 芯片供电VDDA	01:PA3	10/11:PGA输出
	CMPFT = 0x01;  // 00:不滤波	01:1个滤波	02:2个滤波	03:3个滤波
	CMPREF = 0x40; // 内部参考电压
	CMPREF |= 0x2f;
	CMPINT = 0x00;
	CMPFLAG = 0x00;
}
/*****************************************************************************
 函 数 名  : CMP_ISR
 功能描述  : 比较器中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CMP_ISR() interrupt 14
{
	CMP_INT_DISEN;
	SFRPAGE = 0x00;
	CMPINT = 0x00;
	if ((CMPFLAG & 0x03) == 0x03) // 上升沿
	{
		MotorStop();
		mcState = mcFault;
		error_code = overcurrent;
	}
	else
	{
		CMP_INT_EN;
	}
}