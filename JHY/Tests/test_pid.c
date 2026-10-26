/* Regression checks for JHY's corrected firmware PID arithmetic. */
#include "alg_PID.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    Struct_PID p;
    PID_Init(&p, 2.0f, 0.0f, 0.0f, 0.0f, 10.0f, 100.0f,
             .001f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_Enable);
    p.Target = 3.0f;
    PID_TIM_Adjust_PeriodElapsedCallback(&p);
    assert(isfinite(p.Out) && p.Out == 6.0f && p.I_Error == 0.0f);
    p.K_P = 0.0f; p.K_I = 2.0f; p.D_T = 1.0f;
    p.I_Out_Max = 1.0f;
    PID_TIM_Adjust_PeriodElapsedCallback(&p);
    assert(isfinite(p.Out) && fabsf(p.Out) <= 1.00001f);
    p.D_T = 0.0f;
    PID_TIM_Adjust_PeriodElapsedCallback(&p);
    assert(p.Out == 0.0f && p.I_Error == 0.0f);
    p.D_T = .001f; p.Target = NAN;
    PID_TIM_Adjust_PeriodElapsedCallback(&p);
    assert(p.Out == 0.0f && p.I_Error == 0.0f);
    puts("PID regression: Ki=0, post-update integral clamp, invalid dt and NaN PASS");
    return 0;
}
