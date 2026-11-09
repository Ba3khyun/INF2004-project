#include <assert.h>
#include <stdio.h>
#include "../app_program/subsystem2/motion_feedback_math.h"
static float absf(float x) { return x < 0 ? -x : x; }
int main(void)
{
    assert(absf(b2_estimated_turn_deg(-94.2478f, 94.2478f,120.f)-90.f)<1.f);
    assert(absf(b2_estimated_turn_deg(94.2478f,-94.2478f,120.f)+90.f)<1.f);
    assert(b2_estimated_turn_deg(20.f,30.f,0.f)==0.f);
    assert(b2_cap_pwm(20.f,100u)==20.f);
    assert(b2_cap_pwm(20.f,50u)==10.f);
    assert(b2_cap_pwm(20.f,45u)==9.f);
    assert(b2_cap_pwm(20.f,0u)==0.f);
    assert(b2_cap_pwm(20.f,101u)==0.f);
    puts("[B4-B2 HOST] PASS: signed geometry model + bounded PWM cap");
    return 0;
}
