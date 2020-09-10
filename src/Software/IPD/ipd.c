#include "ipd.h"
#include "common.h"
#include "init.h"
#include "pwm.h"
#include "adc.h"
#include "uart.h"
void ADC_PGA()
{
	SFRPAGE = 0x00;
	ADCCON0 &= BIT6F;
	ADCCON0 |= BIT7;
	while (!(ADCCON0 & BIT6))
		;
}
void discharge(void) //放电
{
	GPIO_UH_N;
	GPIO_VH_N;
	GPIO_WH_N;
	GPIO_UL_N;
	GPIO_VL_N;
	GPIO_WL_N;
	Delay_us(200);
}
void position()
{
	HoldParm.PWMDutyCycle = PWM_START_DUTY;
	PWMChangeDuty(HoldParm.PWMDutyCycle);
	PWMOutput();
	PWMSwitchPhase();
	Delay_ms(100);
	PWMPortShut();
	Delay_us(50);
}
void IPD()
{
	u16 U1, U2, V1, V2, W1, W2, U_edge = 0, V_edge = 0, W_edge = 0, time = 100;
	u8 UVW = 0;
	SFRPAGE = 0x00;
	ADCCON0 = 0x10; // 打开ADC，单次模式，不启动ADC转换
	ADCCON0 &= 0xf0; 
	ADCPGAC = 0x13;
	do
	{
		discharge();
		Delay_ms(1);
		GPIO_UH;
		GPIO_VL;
		GPIO_WL;
		Delay_us(time);
		Start_ADCSample(CHL_IBUS);
		U1 = Get_CHL_Value();
		if (U1 < 100)
			ADCPGAC--;
		else if (U1 > 300)
		{
			ADCPGAC++;
		}
		else if (U1 > 200 && U1 < 300)
		{
			time += 10;
		}
	} while (U1 < 100 || U1 > 200);
	discharge();
	GPIO_UL;
	GPIO_VH;
	GPIO_WH;
	Delay_us(time);
	Start_ADCSample(CHL_IBUS);
	U2 = Get_CHL_Value();
	discharge();
	GPIO_VH;
	GPIO_UL;
	GPIO_WL;
	Delay_us(time);
	Start_ADCSample(CHL_IBUS);
	V1 = Get_CHL_Value();
	discharge();
	GPIO_VL;
	GPIO_UH;
	GPIO_WH;
	Delay_us(time);
	Start_ADCSample(CHL_IBUS);
	V2 = Get_CHL_Value();
	discharge();
	GPIO_WH;
	GPIO_UL;
	GPIO_VL;
	Delay_us(time);
	Start_ADCSample(CHL_IBUS);
	W1 = Get_CHL_Value();
	discharge();
	GPIO_WL;
	GPIO_VH;
	GPIO_UH;
	Delay_us(time);
	Start_ADCSample(CHL_IBUS);
	W2 = Get_CHL_Value();
	discharge();
	// Ps("\nU+=", U1);
	// Ps("U-=", U2);
	// Ps("\tV+=", V1);
	// Ps("V-=", V2);
	// Ps("\tW+=", W1);
	// Ps("W-=", W2);
	// Ps("\tdelay=", time);
	// Ps("\tpga=", ADCPGAC & 0x07);
	if (U1 < U2)
	{
		if (U2 - U1 <= 20)
			U_edge = 1;
		else
			UVW |= 0x04;
	}
	else if (U1 - U2 <= 20)
		U_edge = 1;
	if (V1 < V2)
	{
		if (V2 - V1 <= 20)
			V_edge = 1;
		else
			UVW |= 0x02;
	}
	else if (V1 - V2 <= 20)
		V_edge = 1;
	if (W1 < W2)
	{
		if (W2 - W1 <= 20)
			W_edge = 1;
		else
			UVW |= 0x01;
	}
	else if (W1 - W2 <= 20)
		W_edge = 1;
	if (HoldParm.RotorDirection == CW)
	{
		switch (UVW)
		{
		case 5:
			// Ps("5", NONE);
			Halless.Phase = 5;
			break;
		case 1:
			// Ps("1", NONE);
			Halless.Phase = 4;
			break;
		case 3:
			// Ps("3", NONE);
			Halless.Phase = 3;
			break;
		case 2:
			// Ps("2", NONE);
			Halless.Phase = 2;
			break;
		case 6:
			// Ps("6", NONE);
			Halless.Phase = 1;
			break;
		case 4:
			// Ps("4", NONE);
			Halless.Phase = 0;
			break;
		default:
			position();
			break;
		}
	}
	else if (HoldParm.RotorDirection == CCW)
	{
		switch (UVW)
		{
		case 5:
			// Ps("5", NONE);
			Halless.Phase = 0;
			break;
		case 1:
			// Ps("1", NONE);
			Halless.Phase = 1;
			break;
		case 3:
			// Ps("3", NONE);
			Halless.Phase = 2;
			break;
		case 2:
			// Ps("2", NONE);
			Halless.Phase = 3;
			break;
		case 6:
			// Ps("6", NONE);
			Halless.Phase = 4;
			break;
		case 4:
			// Ps("4", NONE);
			Halless.Phase = 5;
			break;
		default:
			position();
			break;
		}
	}
	// if (U_edge | V_edge | W_edge)
	// if (++Halless.Phase > 5)
	// 	Halless.Phase = 0;
}