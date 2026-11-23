#ifndef B2_MOTION_FEEDBACK_MATH_H
#define B2_MOTION_FEEDBACK_MATH_H
#include <stdint.h>
/* Pure C calculations. Direction is INFERRED from commanded motion because
 * GP6/GP7 count A-channel rising edges; no quadrature direction is decoded. */
float b2_estimated_turn_deg(float left_signed_mm, float right_signed_mm,
                            float wheel_track_mm);
/* Scale Buddy 2's existing PWM setting, then clamp the result to 100%.
 * Invalid scales above B2_MAX_PWM_SCALE_PERCENT return zero. */
#define B2_MAX_PWM_SCALE_PERCENT 150U
float b2_scale_pwm(float base_pwm_percent, uint32_t speed_scale_percent);
#endif
