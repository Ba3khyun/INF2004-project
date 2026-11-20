#ifndef B2_MOTION_FEEDBACK_MATH_H
#define B2_MOTION_FEEDBACK_MATH_H
#include <stdint.h>
/* Pure C calculations. Direction is INFERRED from commanded motion because
 * GP6/GP7 count A-channel rising edges; no quadrature direction is decoded. */
float b2_estimated_turn_deg(float left_signed_mm, float right_signed_mm,
                            float wheel_track_mm);
/* A cap is a fraction of Buddy 2's existing PWM setting, not a new base PWM.
 * Invalid percentage (>100) is treated as zero by this pure helper. */
float b2_cap_pwm(float base_pwm_percent, uint32_t speed_cap_percent);
#endif
