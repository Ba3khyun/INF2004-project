#include "vehicle_controller_core.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    vc_core_t vc;
    vc_core_init(&vc);
    assert(vc_core_step(&vc, 0) == VC_OUTPUT_HALT && vc.state == VC_INITIALIZING);
    vc_core_line(&vc, VC_INTENT_FORWARD, 0);
    assert(vc_core_step(&vc, 1) == VC_OUTPUT_HALT); /* IMU not ready */
    vc_core_imu(&vc, VC_IMU_READY, 0, 0);
    assert(vc_core_step(&vc, 5) == VC_OUTPUT_FORWARD && vc.state == VC_DRIVING);
    vc_core_imu(&vc, VC_IMU_HUMP_START, 0, 0);
    assert(vc_core_step(&vc, 6) == VC_OUTPUT_HALT && vc.state == VC_HUMP_CROSSING);
    vc_core_imu(&vc, VC_IMU_HUMP_DONE, 1, 22);
    assert(vc.last_hump_valid && vc.last_hump_mm == 22);
    assert(vc_core_step(&vc, 7) == VC_OUTPUT_FORWARD);
    assert(vc_core_step(&vc, 507) == VC_OUTPUT_HALT && vc.state == VC_LINE_SEARCH);
    vc_core_line(&vc, VC_INTENT_STEER_RIGHT, 600);
    assert(vc_core_step(&vc, 601) == VC_OUTPUT_STEER_RIGHT);
    vc_core_line_visible(&vc, 0);
    assert(vc_core_step(&vc, 602) == VC_OUTPUT_HALT && vc.state == VC_LINE_SEARCH);
    vc_core_line_visible(&vc, 1);
    vc_core_set_safety(&vc, 1, 0);
    assert(vc_core_step(&vc, 602) == VC_OUTPUT_HALT && vc.state == VC_ERROR);
    vc_core_set_safety(&vc, 0, 1);
    vc_core_resume(&vc);
    assert(vc_core_step(&vc, 603) == VC_OUTPUT_HALT && vc.state == VC_ERROR);
    vc_core_set_safety(&vc, 0, 0);
    vc_core_pause(&vc);
    assert(vc_core_step(&vc, 604) == VC_OUTPUT_HALT && vc.state == VC_PAUSED);
    vc_core_resume(&vc);
    assert(vc_core_step(&vc, 605) == VC_OUTPUT_STEER_RIGHT);
    vc_core_imu(&vc, VC_IMU_FAULT, 0, 0);
    assert(vc_core_step(&vc, 606) == VC_OUTPUT_HALT && vc.state == VC_ERROR);
    vc_core_imu(&vc, VC_IMU_READY, 0, 0);
    assert(vc_core_step(&vc, 607) == VC_OUTPUT_STEER_RIGHT);
    vc_core_complete(&vc);
    assert(vc_core_step(&vc, 608) == VC_OUTPUT_HALT && vc.state == VC_MISSION_COMPLETE);
    puts("VC core: all software-only tests passed");
    return 0;
}
