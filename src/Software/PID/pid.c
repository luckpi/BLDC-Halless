#include "pid.h"
#include "pwm.h"
#include "common.h"
PIDREG_T xdata pi_spd;
void PID_init(void)
{
    pi_spd.Kp = Speed_Kp;
    pi_spd.Ki = Speed_Ki;
    pi_spd.Kd = Speed_Kd;
    pi_spd.Up = 0;
    pi_spd.Ui = 0;
    pi_spd.Ud = 0;
    pi_spd.Fdb = 0;
    pi_spd.Ref = HoldParm.Set_RPM;
    pi_spd.Err = 0;
    pi_spd.L_Err = 0;
    pi_spd.Err_sum = 0;
    pi_spd.Out = 0;
    pi_spd.OutMax = 5;
    pi_spd.OutMin = -pi_spd.OutMax;
    // pi_spd.UMAX = 60000;
}
void PID_CLAC(PIDREG_T *PID)
{
    PID->Err = PID->Ref - PID->Fdb;
    PID->Err_sum += PID->Err;
    PID->Up = PID->Kp * PID->Err;
    PID->Ui = PID->Ki * PID->Err_sum;
    // UP16LIMIT(PID->Err_sum, PID->UMAX, PID->Ui);
    PID->Ud = PID->Kd * (PID->Err - PID->L_Err);
    PID->Out = PID->Up + PID->Ui + PID->Ud;
    UP16LIMIT(PID->Out, PID->OutMax, PID->OutMin);
}