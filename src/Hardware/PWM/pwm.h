#ifndef _PWM_H
#define _PWM_H
#include "typedef.h"
#include "common.h"
#define PWMSYSCLK								(73728000)	// PWM时钟
#define SYSCLK									(24576000)	// 除PWM外的模块时钟

#define PWM_FRE							(20000)	//PWM的频率
#define PWM_DEADLINE					(147)	// 死区设置
#define PWM_FRE_SETATA					((PWMSYSCLK / PWM_FRE) >> 1)   
#define PWM_DUTYCYCLE_95				((PWM_FRE_SETATA / 20) * 19)	// PWM的占空比  95%
#define PWM_DUTYCYCLE_75				((PWM_FRE_SETATA / 4) * 3)	// PWM的占空比  75%
#define PWM_DUTYCYCLE_50				(PWM_FRE_SETATA / 2)	// PWM的占空比  50%
#define PWM_DUTYCYCLE_25				(PWM_FRE_SETATA / 4)	// PWM的占空比  25%
#define PWM_DUTYCYCLE_20				(PWM_FRE_SETATA / 5)	 // PWM的占空比  20%
#define PWM_DUTYCYCLE_15				((PWM_FRE_SETATA / 20) * 3)	 // PWM的占空比  15%
#define PWM_DUTYCYCLE_10				(PWM_FRE_SETATA / 10)	 // PWM的占空比  10%
#define PWM_DUTYCYCLE_05				(PWM_FRE_SETATA / 20)	 // PWM的占空比  5%
#define PWM_DUTYCYCLE_00				(1)	// PWM的占空比  0%
#define POLE_PAIRS						(8)	// 极对数

#define TIMERFREQUENCY				    (8)	// Timer时钟预分频系数[1/8/64/256]，最低转速对应的60度电周期的时间
#define SPEEDFACTOR                     (SYSCLK / 2 / 3 / POLE_PAIRS / TIMERFREQUENCY * 60) // 计算转速的系数
#define PWM_START_DUTY					(PWM_DUTYCYCLE_10)
#define PWM_MIN_DUTY					(PWM_DUTYCYCLE_05)	// 最小占空比
#define PWM_MAX_DUTY					PWM_FRE_SETATA// 最大占空比

#define PWM_UN_VN_WN       \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_CCENR = 0x00; \
        TIM0_CCPS = 0x00;  \
    } // PWM模式1
#define PWM_UL_VL_WL       \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_CCENR = 0x15; \
        TIM0_CCPS = 0x00;  \
    } // PWM下桥臂输出pwm
#define PWM_EGR_UPDATA   \
    {                    \
        SFRPAGE = 0x01;  \
        TIM0_EGR = 0x04; \
    } // 更新 TIM_CCENR、TIM_CCPS 和 TIM_CCxMR.OCxMS

// 非同步整流[目前使用方法]
// 下管NMOS使用PWM控制
#define PWM_U_H             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x20; \
        TIM0_CCPS |= 0x00;  \
    }
#define PWM_U_L             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x00; \
        TIM0_CCPS |= 0x10;  \
    }
#define PWM_U_N             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x00; \
        TIM0_CCPS |= 0x00;  \
    }

// V
#define PWM_V_H             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x08; \
        TIM0_CCPS |= 0x00;  \
    }
#define PWM_V_L             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x00; \
        TIM0_CCPS |= 0x04;  \
    }
#define PWM_V_N             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x00; \
        TIM0_CCPS |= 0x00;  \
    }

// W
#define PWM_W_H             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x02; \
        TIM0_CCPS |= 0x00;  \
    } // PWM模式1
#define PWM_W_L             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x0; \
        TIM0_CCPS |= 0x01;  \
    } // 输出有效电平
#define PWM_W_N             \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_CCENR |= 0x00; \
        TIM0_CCPS |= 0x00;  \
    } // 输出无效电平

#define PWM_START          \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_CNTEN = 0x01; \
    }
#define PWM_STOP           \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_CNTEN = 0x00; \
    }

#define PWM_OUT_MOEN       \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_BRKC |= BIT7; \
    }
#define PWM_OUT_MODISEN     \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_BRKC &= BIT7F; \
    }

#define PWM_BRK_EN         \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_BRKC |= BIT4; \
    }
#define PWM_BRK_DISEN       \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_BRKC &= BIT4F; \
    }

#define PWM_BRK_HIGH        \
    {                       \
        SFRPAGE = 0x01;     \
        TIM0_BRKC &= BIT5F; \
    }
#define PWM_BRK_LOW        \
    {                      \
        SFRPAGE = 0x01;    \
        TIM0_BRKC |= BIT5; \
    }

#define GPIO_UH  \
    {            \
        PA5 = 1; \
    }
#define GPIO_UL  \
    {            \
        PB0 = 1; \
    }

#define GPIO_VH  \
    {            \
        PA6 = 1; \
    }
#define GPIO_VL  \
    {            \
        PB1 = 1; \
    }

#define GPIO_WH  \
    {            \
        PA7 = 1; \
    }
#define GPIO_WL  \
    {            \
        PB7 = 1; \
    }

#define GPIO_UH_N \
    {             \
        PA5 = 0;  \
    }
#define GPIO_UL_N \
    {             \
        PB0 = 0;  \
    }

#define GPIO_VH_N \
    {             \
        PA6 = 0;  \
    }
#define GPIO_VL_N \
    {             \
        PB1 = 0;  \
    }

#define GPIO_WH_N \
    {             \
        PA7 = 0;  \
    }
#define GPIO_WL_N \
    {             \
        PB7 = 0;  \
    }  
extern void PWM_Init();
extern void PWMOutput();
extern void PWMPortShut();
extern void PWMSwitchPhase();
extern void PWMChangeDuty(u16 u16data);
extern void PWMPortBrake();
extern void PWMSwitchPhase();
extern void halless_to_phase();
#endif