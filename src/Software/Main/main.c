#include "common.h"
#include "typedef.h"
#include "uart.h"
#include "halless.h"
#include "init.h"
#include "pwm.h"
#include "control.h"
#include "adc.h"
#include "protect.h"
#include "cmp.h"
#include "pid.h"
#include "timer.h"
void main()
{
    PowerupParaInit();
    Crpm_Init();
    Port_Init();
    WDT_Init();
    Interrupt_Init();
    Common_Init();
    UART_Init();
    PWM_Init();
    ADC_Init();
    Timer2_Init();
    Timer0_Init();
    CMP_Init();
    while (1)
    {
        MotorControl(); //电机控制
        switch (mcState)
        {
        case mcRun:
            if (ADCSample.Voltage)
                Protect_Voltage();
            // Ps("BAT=", ADCSample.Voltage);
            // Ps("adc=", ADCSample.Average);
            // Protect_Current();
            break;
        default:
            break;
        }
    }
}