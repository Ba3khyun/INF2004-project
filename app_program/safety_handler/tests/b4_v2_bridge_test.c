/* Pure tests: no queue receives, motor posts or live safety writes. */
#include "../../app.h"
#include "../../vehicle_controller/b4_input_bridge.h"
#include "../../subsystem4/imu_math.h"
#include "../../subsystem5/subsystem5.h"
#include "b4_v2_bridge_test.h"
#define B4_V2_CHECK(c) do { if (!(c)) failures++; } while (0)
int b4_v2_bridge_test_run(void)
{
    int failures = 0;
    unsigned i;
    b4_input_bridge_t b;
    imu_pickup_detector_t p;
    b4_input_bridge_init(&b);
    B4_V2_CHECK(!b4_input_bridge_halt(&b));
    b4_input_bridge_remote(&b, CMD_PAUSE);
    B4_V2_CHECK(b.paused);
    b4_input_bridge_remote(&b, CMD_RESUME);
    B4_V2_CHECK(!b.paused);
    b4_input_bridge_obstacle(&b, S5_ST_PROFILING, 1u, S5_ACT_STOP);
    B4_V2_CHECK(b.obstacle_hold && b4_input_bridge_halt(&b));
    b4_input_bridge_obstacle(&b, S5_ST_DECIDING, 1u, S5_ACT_TURN_LEFT);
    B4_V2_CHECK(b.obstacle_hold);
    b4_input_bridge_refresh(&b, 1);
    B4_V2_CHECK(b.obstacle_hold); /* no release on mere direction */
    b4_input_bridge_obstacle(&b, S5_ST_MONITORING, 0u, S5_ACT_CONTINUE);
    b4_input_bridge_refresh(&b, 0);
    B4_V2_CHECK(b.obstacle_hold); /* stale/lost line */
    b4_input_bridge_refresh(&b, 1);
    B4_V2_CHECK(!b.obstacle_hold);
    b4_input_bridge_obstacle_flag(&b, 0, 1);
    B4_V2_CHECK(b4_input_bridge_halt(&b));
    b4_input_bridge_obstacle_flag(&b, 0, 0);
    B4_V2_CHECK(!b4_input_bridge_halt(&b));
    b4_input_bridge_remote(&b, CMD_ABORT);
    b4_input_bridge_remote(&b, CMD_RESUME);
    B4_V2_CHECK(b.remote_abort_latched && b4_input_bridge_halt(&b));
    b4_input_bridge_init(&b);
    b4_input_bridge_pickup(&b);
    b4_input_bridge_remote(&b, CMD_RESUME);
    B4_V2_CHECK(b.pickup_terminal && b4_input_bridge_halt(&b));

    /* Measured track hump reference heights (not proof of sensor accuracy). */
    B4_V2_CHECK(app_tq_check(TQ_FIELD_HUMP, 6) == APP_OK);
    B4_V2_CHECK(app_tq_check(TQ_FIELD_HUMP, 24) == APP_OK);
    imu_pickup_detector_init(&p);
    for (i=0u;i<30u;i++)
        B4_V2_CHECK(imu_pickup_detector_step(&p, 1.00f, 14.0f, 2.0f) == IMU_PICKUP_NONE);
    for (i=0u;i<IMU_PICKUP_TILT_SAMPLES-1u;i++)
        B4_V2_CHECK(imu_pickup_detector_step(&p, 1.0f, 70.0f, 0.0f) == IMU_PICKUP_NONE);
    B4_V2_CHECK(imu_pickup_detector_step(&p, 1.0f, 70.0f, 0.0f) == IMU_PICKUP_OVERTURN);
    B4_V2_CHECK(imu_pickup_detector_step(&p, 1.0f, 0.0f, 0.0f) == IMU_PICKUP_OVERTURN);
    imu_pickup_detector_init(&p);
    for (i=0u;i<IMU_PICKUP_LOW_G_SAMPLES-1u;i++)
        B4_V2_CHECK(imu_pickup_detector_step(&p, 0.5f, 0.0f, 0.0f) == IMU_PICKUP_NONE);
    B4_V2_CHECK(imu_pickup_detector_step(&p, 0.5f, 0.0f, 0.0f) == IMU_PICKUP_LOW_ACCEL);
    /* Separate, conservative dynamic level-lift model. Motor/flag writes
     * are NOT exercised here; actual stop still requires Buddy 2 handoff. */
    imu_pickup_detector_init(&p);
    p.baseline_vertical_g = 1.056f;
    for (i=0u; i<IMU_LIFT_ARM_SAMPLES+1u; i++) {
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.056f, 0.0f, 0.0f, 0.0f)
                    == IMU_PICKUP_NONE);
    }
    /* A single bump or sample burst below the debounce must not trigger. */
    for (i=0u; i<IMU_LIFT_UP_SAMPLES-1u; i++) {
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.20f, 0.0f, 0.0f, 0.0f)
                    == IMU_PICKUP_NONE);
    }
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.056f, 0.0f, 0.0f, 0.0f)
                == IMU_PICKUP_NONE);
    /* Valid stationary upward movement: 4 samples at +0.14g. */
    for (i=0u; i<IMU_LIFT_UP_SAMPLES-1u; i++) {
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.20f, 0.0f, 0.0f, 0.0f)
                    == IMU_PICKUP_NONE);
    }
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.20f, 0.0f, 0.0f, 0.0f)
                == IMU_PICKUP_LIFT_MOTION);
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.056f, 0.0f, 0.0f, 0.0f)
                == IMU_PICKUP_LIFT_MOTION); /* sticky */
    imu_pickup_detector_init(&p);
    p.baseline_vertical_g = 1.056f;
    /* Normal wheel travel arms the detector. A short bump resets, then a
     * sustained upward pulse with the same travel ends the run. */
    for (i=0u; i<IMU_LIFT_ARM_SAMPLES; i++)
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.056f, 3.0f, 0.0f, 0.0f)
                    == IMU_PICKUP_NONE);
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.25f, 3.0f, 0.0f, 0.0f)
                == IMU_PICKUP_NONE);
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.056f, 3.0f, 0.0f, 0.0f)
                == IMU_PICKUP_NONE);
    for (i=1u; i<IMU_LIFT_UP_SAMPLES; i++)
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.25f, 3.0f, 0.0f, 0.0f)
                    == IMU_PICKUP_NONE);
    B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.25f, 3.0f, 0.0f, 0.0f)
                == IMU_PICKUP_LIFT_MOTION);
    imu_pickup_detector_init(&p);
    p.baseline_vertical_g = 1.056f;
    for (i=0u; i<IMU_LIFT_ARM_SAMPLES+10u; i++) {
        B4_V2_CHECK(imu_pickup_lift_motion_step(&p, 1.25f, 0.0f, 25.0f, 0.0f)
                    == IMU_PICKUP_NONE); /* steep tilt disarms heuristic */
    }
    tm_printf((UB*)"[B4-V2] remote pause/resume/abort; obstacle persistent hold; "
                "pickup/overturn software model (no motor output) failures=%d\n", failures);
    return failures;
}
