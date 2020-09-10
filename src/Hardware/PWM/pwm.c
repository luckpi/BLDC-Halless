#include "pwm.h"
#include "halless.h"
void PWM_Init()
{
	SFRPAGE = 0x01;
	TIM0_CONR1 = 0x00;
	TIM0_CCPS = 0x00;
	TIM0_ARRH = PWM_FRE_SETATA >> 8;
	TIM0_ARRL = PWM_FRE_SETATA;
	TIM0_PSCH = 0x00;
	TIM0_PSCL = 0x00;
	TIM0_RCR = 0x00;
	TIM0_PHACON = 0x00;
	TIM0_PHASEH = 0x00;
	TIM0_PHASEL = 0x00;
	TIM0_CONR0 = 0x90; // CCPE
	TIM0_CONR2 = 0x05; // 打开TIM0_ARR寄存器预装载功能，中央对齐模式2，计数器向上计数使更新输出通道的输出比较中断标志位
	TIM0_CONR3 = 0x01; // 上溢不产生更新事件，不产生更新中断，下溢产生更新事件，不产生更新中断
	TIM0_CC0MR = 0x68; // PWM0模式1
					   // 中央对齐计数模式下：向上计数时，若TIM0_CNT < TIM0_CC0R，OC0REF为有效电平（ OC0REF=1），否则为无效电平（ OC0REF=0）
					   // 向下计数时，若 TIM0_CNT > TIM0_CC0R，OC0REF 为无效电平（OC0REF=0），否则为有效电平（OC0REF=1）
	TIM0_CC1MR = 0x68;
	TIM0_CC2MR = 0x68;
	TIM0_CC0RH = PWM_DUTYCYCLE_00 >> 8;
	TIM0_CC0RL = PWM_DUTYCYCLE_00;
	TIM0_CC1RH = PWM_DUTYCYCLE_00 >> 8;
	TIM0_CC1RL = PWM_DUTYCYCLE_00;
	TIM0_CC2RH = PWM_DUTYCYCLE_00 >> 8;
	TIM0_CC2RL = PWM_DUTYCYCLE_00;
	TIM0_DTC = PWM_DEADLINE; //死区时间
	TIM0_IER0 = 0x00;
	TIM0_IER1 = 0x00;
	TIM0_BRKC = 0x86; // 使用CCENR&CCPS换相时，[bit2]ROSS必须配置，[bit1]IOSS是为刹车配置
	PWM_OUT_MODISEN;
	TIM0_ISR0 = 0x00;
	TIM0_ISR1 = 0x00;
	TIM0_EGR = 0x84; // 产生更新事件
	PWM_START;
}

/*****************************************************************************
 函 数 名  : PWMPortBrake
 功能描述  : PWM管脚下桥臂输出有效电平
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PWMPortBrake()
{
	PWM_UN_VN_WN;
	PWM_EGR_UPDATA;
	PWM_OUT_MODISEN;

	GPIO_UH_N;
	GPIO_UL;
	GPIO_VH_N;
	GPIO_VL;
	GPIO_WH_N;
	GPIO_WL;

	SFRPAGE = 0x02;
	PB0CTRL = BIT1; // UL PWM0_CH2
	PA5CTRL = BIT1; // UH PWM0_CH2N
	PB7CTRL = BIT1; // WL PWM0_CH0
	PA7CTRL = BIT1; // WH PWM0_CH0N
	PB1CTRL = BIT1; // VL PWM0_CH1
	PA6CTRL = BIT1; // VH PWM0_CH1N
}

/*****************************************************************************
 函 数 名  : PWMOutput
 功能描述  : PWM0波形输出
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PWMOutput()
{
	PWM_UN_VN_WN;
	PWM_EGR_UPDATA;
	PWM_OUT_MOEN;

	SFRPAGE = 0x02;
	PB0CTRL = 0xc0; // UL PWM0_CH2
	PA5CTRL = 0xc0; // UH PWM0_CH2N
	PB7CTRL = 0xc0; // WL PWM0_CH0
	PA7CTRL = 0xc0; // WH PWM0_CH0N
	PB1CTRL = 0xc0; // VL PWM0_CH1
	PA6CTRL = 0xc0; // VH PWM0_CH1N
}

/*****************************************************************************
 函 数 名  : PWMPortShut
 功能描述  : PWM管脚关闭，输出无效电平
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PWMPortShut()
{
	PWM_UN_VN_WN;
	PWM_EGR_UPDATA;
	PWM_OUT_MODISEN;

	GPIO_UH_N;
	GPIO_UL_N;
	GPIO_VH_N;
	GPIO_VL_N;
	GPIO_WH_N;
	GPIO_WL_N;

	SFRPAGE = 0x02;
	PB0CTRL = BIT1; // UL PWM0_CH2
	PA5CTRL = BIT1; // UH PWM0_CH2N
	PB7CTRL = BIT1; // WL PWM0_CH0
	PA7CTRL = BIT1; // WH PWM0_CH0N
	PB1CTRL = BIT1; // VL PWM0_CH1
	PA6CTRL = BIT1; // VH PWM0_CH1N
}

void halless_to_phase()
{
	if (HoldParm.RotorDirection == CW)
	{
		switch (Halless.HallessState)
		{
		case 5:
			Halless.Phase = 5;
			break;
		case 1:
			Halless.Phase = 4;
			break;
		case 3:
			Halless.Phase = 3;
			break;
		case 2:
			Halless.Phase = 2;
			break;
		case 6:
			Halless.Phase = 1;
			break;
		case 4:
			Halless.Phase = 0;
			break;
		default:
			break;
		}
	}
	else if (HoldParm.RotorDirection == CCW)
	{
		switch (Halless.HallessState)
		{
		case 5:
			Halless.Phase = 0;
			break;
		case 1:
			Halless.Phase = 1;
			break;
		case 3:
			Halless.Phase = 2;
			break;
		case 2:
			Halless.Phase = 3;
			break;
		case 6:
			Halless.Phase = 4;
			break;
		case 4:
			Halless.Phase = 5;
			break;
		default:
			break;
		}
	}
}
/*****************************************************************************
 函 数 名  : PWMSwitchPhase
 功能描述  : PWM0换相函数[CW:AB-AC-BC-BA-CA-CB	CCW:AB-CB-CA-BA-BC-AC]
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void PWMSwitchPhase()
{
	PWM_UN_VN_WN; //通道全部关闭
	if (HoldParm.RotorDirection == CW)
	{
		switch (Halless.Phase)
		{
		case 0: // U+ V-
			PWM_U_H;
			PWM_V_L;
			PWM_W_N;
			break;
		case 1: // U+ W-
			PWM_U_H;
			PWM_V_N;
			PWM_W_L;
			break;
		case 2: // V+ W-
			PWM_U_N;
			PWM_V_H;
			PWM_W_L;
			break;
		case 3: // V+ U-
			PWM_U_L;
			PWM_V_H;
			PWM_W_N;
			break;
		case 4: // W+ U-
			PWM_U_L;
			PWM_V_N;
			PWM_W_H;
			break;
		case 5: // W+ V-
			PWM_U_N;
			PWM_V_L;
			PWM_W_H;
			break;
		default:
			break;
		}
	}
	else if (HoldParm.RotorDirection == CCW)
	{
		switch (Halless.Phase)
		{
		case 0: // U+ V-
			PWM_U_H;
			PWM_V_L;
			PWM_W_N;
			break;
		case 1: // W+ V-
			PWM_U_N;
			PWM_V_L;
			PWM_W_H;
			break;
		case 2: // W+ U-
			PWM_U_L;
			PWM_V_N;
			PWM_W_H;
			break;
		case 3: // V+ U-
			PWM_U_L;
			PWM_V_H;
			PWM_W_N;
			break;
		case 4: // V+ W-
			PWM_U_N;
			PWM_V_H;
			PWM_W_L;
			break;
		case 5: // U+ W-
			PWM_U_H;
			PWM_V_N;
			PWM_W_L;
			break;
		default:
			break;
		}
	}
	PWM_EGR_UPDATA; // 更新 TIM0_CCENR、TIM0_CCPS 和 TIM0_CCxMR.OCxMS
}

/*****************************************************************************
 函 数 名  : PWMChangeDuty
 功能描述  : PWM0占空比更新
 输入参数  : u16data
 输出参数  : void
*****************************************************************************/
void PWMChangeDuty(u16 u16data)
{
	SFRPAGE = 0x01;
	TIM0_CC0RH = u16data >> 8;
	TIM0_CC0RL = u16data;
	TIM0_CC1RH = u16data >> 8;
	TIM0_CC1RL = u16data;
	TIM0_CC2RH = u16data >> 8;
	TIM0_CC2RL = u16data;
}
