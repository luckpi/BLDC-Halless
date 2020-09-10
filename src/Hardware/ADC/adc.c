#include "adc.h"
#include "control.h"
#include "halless.h"
#include "uart.h"
#include "protect.h"
#include "common.h"
#include "pid.h"
#include "pwm.h"
/*****************************************************************************
 函 数 名  : ADC_Init
 功能描述  : ADC初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADC_Init()
{
    SFRPAGE = 0x02;
    ADC_CLK_DIV = 0x02; // AD时钟分频[7分频]	0x06
    SFRPAGE = 0x00;
    ADCETC = 0x00;
    ADCCON1 = 0x00; // 无符号整数，正常模式，内部2.5V参考电压
    ADCPGAC = 0x00; // 不使能校准，正常工作模式
    ADC_INT_DISEN;
}
/*****************************************************************************
 函 数 名  : ADCExtTrigEnable
 功能描述  : 使能ADC外部触发采样
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void ADCExtTrigEnable()
{
    SFRPAGE = 0x00;
    ADCCON0 = 0x10; // 打开ADC，单次模式，不启动ADC转换
    ADCCON0 &= 0xf0;
    ADCCON0 |= CHL_IBUS;
    CMP_PGA_I;     // PGA放大
    ADCETC = 0x75; // 外部触发事件选择TIM0_TRGO，触发事件使能，上升沿使能，下降沿使能
    ADC_INT_EN;
}
void Start_ADCSample(u8 ADC_CHL)
{
    ADC_CHL_CLEAR;            // 清除通道
    ADCCON0 |= ADC_CHL;       // 切换通道
    ADCCON0 |= BIT7;          // 启动ADC转换
    while (!(ADCCON0 & BIT6)) // 等待转换完成
        ;
    ADC_IF_CLEAR; // 清除ADC中断标识位
}
void Switch_ADC_CHL(u8 ADC_CHL)
{
    ADC_CHL_CLEAR;      // 清除通道
    ADCCON0 |= ADC_CHL; // 切换通道
}
u16 Get_CHL_Value()
{
    u16 ADCSample_Value;
    ADCSample_Value = (ADCRH << 8) + ADCRL;
    return (ADCSample_Value);
}
/*****************************************************************************
函 数 名  	       : GetPB4VoltageValue
功能描述  	: 获得PB4电压
输入参数  	: null
输出参数  	: null
返 回 值 		: null


修改历史      :
1.日    期   	       :
2.作    者   	       :
3.修改内容   : 
*****************************************************************************/
u16 GetVoltageValue()
{
    u16 ADC_Value = 0;

    SFRPAGE = 0x00;
    ADCCON0 = 0x04      // PB4
              | 0 << 5  //单次模式
              | 1 << 4; //打开ADC
    ADCCON1 = 0 << 4;   //内部2.5V参考电压

    ADCCON0 |= 1 << 7; //启动ADC转换
    while (1)
    {
        if ((ADCCON0 & 0x40) == 0x40)
        {
            ADCCON0 &= 0xbf;
            ADC_Value = (ADCRH << 8) + ADCRL;
            break;
        }
    }
    return ADC_Value;
}