#include <stdio.h>
#include "../app_program/vehicle_controller/vehicle_controller_motion_adapter.h"

static int fails;
#define T(cond) do { if (!(cond)) { printf("FAIL line %d\n", __LINE__); fails++; } } while (0)
int main(void)
{
    vma_codes_t codes = {'f','a','q','l','r','H'};
    vma_inputs_t good = {1,1,0,0,1,1,1};
    vma_inputs_t in;
    vma_result_t r;
    vc_core_t c;
    const vc_output_t out[] = {VC_OUTPUT_FORWARD, VC_OUTPUT_STEER_LEFT,
        VC_OUTPUT_STEER_RIGHT, VC_OUTPUT_PIVOT_LEFT, VC_OUTPUT_PIVOT_RIGHT};
    const char chars[] = {'f','a','q','l','r'};
    unsigned k;
    for (k=0; k<5; ++k) {
        r = vma_preview(out[k], &good, &codes);
        T(r.candidate_command == (unsigned char)chars[k]);
        T(r.safe_command == 'H' && r.model_ready && r.blockers == 0);
    }
    in=good; in.safety_read_ok=0;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_SAFETY_READ) && !r.model_ready && r.safe_command=='H');
    in=good; in.imu_healthy=0;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_IMU) && !r.model_ready);
    in=good; in.safety_request_active=1;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_SAFETY_REQ) && !r.model_ready);
    in=good; in.estop_latched=1;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_ESTOP) && !r.model_ready);
    in=good; in.obstacle_source_verified=0;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_B5_MISSING) && !r.model_ready);
    in=good; in.live_estop_verified=0;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_STOP_UNVERIFIED) && !r.model_ready);
    in=good; in.motor_handoff_approved=0;
    r=vma_preview(VC_OUTPUT_FORWARD,&in,&codes);
    T((r.blockers & VMA_BLOCK_HANDOFF_PENDING) && !r.model_ready);
    r=vma_preview((vc_output_t)99,&good,&codes);
    T((r.blockers & VMA_BLOCK_BAD_OUTPUT) && r.candidate_command=='H');
    r=vma_preview(VC_OUTPUT_HALT,&good,&codes);
    T(r.candidate_command=='H' && r.safe_command=='H' && !r.model_ready);
    r=vma_preview(VC_OUTPUT_FORWARD,0,&codes);
    T(r.blockers & VMA_BLOCK_BAD_INPUT);
    vc_core_init(&c);
    vc_core_line(&c, VC_INTENT_FORWARD, 100);
    T(vc_core_step(&c, 101)==VC_OUTPUT_HALT);
    vc_core_imu(&c, VC_IMU_READY, 0, 0);
    T(vc_core_step(&c, 102)==VC_OUTPUT_FORWARD);
    vc_core_pause(&c);
    T(vc_core_step(&c, 103)==VC_OUTPUT_HALT);
    vc_core_resume(&c);
    vc_core_set_safety(&c,1,0);
    T(vc_core_step(&c,104)==VC_OUTPUT_HALT);
    vc_core_set_safety(&c,0,1);
    T(vc_core_step(&c,105)==VC_OUTPUT_HALT);
    vc_core_set_safety(&c,0,0);
    vc_core_line_visible(&c,0);
    T(vc_core_step(&c,106)==VC_OUTPUT_HALT);
    vc_core_line_visible(&c,1);
    T(vc_core_step(&c,602)==VC_OUTPUT_HALT);
    printf("P5A HOST %s (%d failures)\n",fails?"FAIL":"PASS",fails);
    return fails?1:0;
}
