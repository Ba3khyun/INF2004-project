/* Firmware self-test: pure-model tests + live read-only event tap sanity.
 * Does not move motors, simulate live safety flags, or drain imu_evt_q. */
#include "../../app.h"
#include "../../vehicle_controller/terrain_turn_logic.h"
#include "../../subsystem4/subsystem4.h"
#include "terrain_turn_integration_test.h"
#define B4_TCHECK(x) do { if (!(x)) failures++; } while (0)
int terrain_turn_integration_test_run(void)
{
    int failures = 0;
    b4_turn_checker_t c = {0};
    float diff = 0.0f;
    UW starts0 = s4_imu_hump_start_count();
    UW done0 = s4_imu_hump_done_word();
    B4_TCHECK(b4_turn_wrap_signed(20.0f - 350.0f) == 30.0f);
    B4_TCHECK(b4_turn_wrap_signed(350.0f - 20.0f) == -30.0f);
    B4_TCHECK(b4_turn_begin(&c, 350.0f, 45.0f, 1000u, 3000u));
    B4_TCHECK(b4_turn_check(&c, 44.0f, 34.0f, 1, 1800u, &diff) == B4_TURN_COMPLETE_AGREED);
    B4_TCHECK(diff <= 10.0f);
    B4_TCHECK(b4_turn_check(&c, 45.0f, 75.0f, 1, 1800u, &diff) == B4_TURN_TRACKING);
    B4_TCHECK(b4_turn_check(&c, 45.0f, 75.0f, 1, 4100u, &diff) == B4_TURN_COMPLETE_ENCODER_FALLBACK);
    B4_TCHECK(b4_turn_check(&c, 20.0f, 75.0f, 1, 4100u, &diff) == B4_TURN_TIMEOUT);
    B4_TCHECK(b4_turn_check(&c, 45.0f, 0.0f, 0, 4100u, &diff) == B4_TURN_COMPLETE_ENCODER_FALLBACK);
    B4_TCHECK(b4_turn_begin(&c, 20.0f, -90.0f, 0u, 3000u));
    B4_TCHECK(b4_turn_check(&c, -90.0f, 290.0f, 1, 500u, &diff) == B4_TURN_COMPLETE_AGREED);
    B4_TCHECK(b4_terrain_request(1, 1, 1) == B4_TERRAIN_SLOW_REQUEST);
    B4_TCHECK(b4_terrain_request(1, 0, 1) == B4_TERRAIN_HALT);
    B4_TCHECK(b4_terrain_request(1, 1, 0) == B4_TERRAIN_HALT);
    B4_TCHECK(b4_terrain_request(0, 1, 1) == B4_TERRAIN_NORMAL);
    tm_printf((UB*)"[B4-TERRAIN] start_count=%u done_word=0x%x (READ-ONLY, NO QUEUE RECEIVE)\n",
              (UINT)starts0, (UINT)done0);
    tm_printf((UB*)"[B4-TERRAIN] fused 45/90 deg, wrap, disagreement, timeout, IMU fallback; speed REQUEST only\n");
    return failures;
}
