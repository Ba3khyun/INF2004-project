#include "motion_feedback_math.h"
#define B2_RAD_TO_DEG 57.2957795f
float b2_estimated_turn_deg(float left_signed_mm, float right_signed_mm,
                            float wheel_track_mm)
{
    if (wheel_track_mm <= 0.0f) return 0.0f; /* invalid geometry: caller must check */
    return ((right_signed_mm - left_signed_mm) / wheel_track_mm) * B2_RAD_TO_DEG;
}
float b2_scale_pwm(float base_pwm_percent, uint32_t speed_scale_percent)
{
    float scaled_pwm;
    if (speed_scale_percent > B2_MAX_PWM_SCALE_PERCENT ||
        base_pwm_percent <= 0.0f) return 0.0f;
    if (base_pwm_percent > 100.0f) base_pwm_percent = 100.0f;
    scaled_pwm = base_pwm_percent * (float)speed_scale_percent / 100.0f;
    return (scaled_pwm > 100.0f) ? 100.0f : scaled_pwm;
}
