#include "motion_feedback_math.h"
#define B2_RAD_TO_DEG 57.2957795f
float b2_estimated_turn_deg(float left_signed_mm, float right_signed_mm,
                            float wheel_track_mm)
{
    if (wheel_track_mm <= 0.0f) return 0.0f; /* invalid geometry: caller must check */
    return ((right_signed_mm - left_signed_mm) / wheel_track_mm) * B2_RAD_TO_DEG;
}
float b2_cap_pwm(float base_pwm_percent, uint32_t speed_cap_percent)
{
    if (speed_cap_percent > 100u || base_pwm_percent <= 0.0f) return 0.0f;
    if (base_pwm_percent > 100.0f) base_pwm_percent = 100.0f;
    return base_pwm_percent * (float)speed_cap_percent / 100.0f;
}
