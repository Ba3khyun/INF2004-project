/* Phase 4E: in-memory producer -> ESTOP bridge -> consumer model.
 * This tests the interface contract, NOT live RTOS scheduling or physical braking.
 * No Pico I/O, kernel objects, or motor commands occur here.
 */
#include "safety_handoff_test.h"
#include "../safety_estop_bridge.h"

/* Simulates a shared flag word. Only these test functions mutate it. */
static void model_assert(seb_state_t *bridge, uint32_t *flags, int read_ok)
{
    if (seb_step(bridge, read_ok ? *flags : 0u, read_ok)) {
        *flags |= SEB_ESTOP; /* simulate tk_set_flg: OR bits, don't clobber */
    }
}

/* Model of Buddy 2's safety_estop_latched(): fail closed on read error. */
static int model_motion_blocked(uint32_t flags, int read_ok)
{
    return !read_ok || ((flags & SEB_ESTOP) != 0u);
}

int safety_phase4e_test_run(void)
{
    int fail = 0;
    uint32_t flags;
    seb_state_t b;

    /* A healthy boot with no requests must not assert an ESTOP. */
    seb_init(&b); flags = 0;
    model_assert(&b, &flags, 1);
    if (flags != 0 || model_motion_blocked(flags, 1)) fail++;

    /* Each source must cause an assert without wiping the other bits. */
    { uint32_t sources[] = {SEB_COLLISION, SEB_OBSTACLE, SEB_REMOTE_ABORT};
      unsigned int i;
      for (i = 0; i < sizeof(sources)/sizeof(sources[0]); ++i) {
          seb_init(&b);
          flags = sources[i] | (1u << 12); /* unrelated flag must survive */
          model_assert(&b, &flags, 1);
          if (!model_motion_blocked(flags, 1)) fail++;
          if ((flags & (1u << 12)) == 0) fail++;
          if ((b.reasons & sources[i]) == 0) fail++;
      }
    }

    /* Sticky property: original request disappears, ESTOP stays asserted. */
    seb_init(&b); flags = SEB_COLLISION;
    model_assert(&b, &flags, 1);
    flags &= ~SEB_COLLISION;
    model_assert(&b, &flags, 1);
    if (!model_motion_blocked(flags, 1) || !b.sticky) fail++;

    /* An external erroneous clear is repaired at the NEXT bridge tick. */
    flags &= ~SEB_ESTOP;
    if (model_motion_blocked(flags, 1)) fail++; /* gap: not protected yet */
    model_assert(&b, &flags, 1);
    if (!model_motion_blocked(flags, 1)) fail++;

    /* Both producer-side read error and consumer-side read error fail closed. */
    seb_init(&b); flags = 0;
    model_assert(&b, &flags, 0);
    if (!model_motion_blocked(flags, 1) || !b.read_failed) fail++;
    if (!model_motion_blocked(0, 0)) fail++;

    /* No request may be resurrected by unrelated flag bits. */
    seb_init(&b); flags = (1u << 14);
    model_assert(&b, &flags, 1);
    if (flags & SEB_ESTOP) fail++;

    /* Existing ESTOP must remain asserted even without request flags. */
    seb_init(&b); flags = SEB_ESTOP;
    model_assert(&b, &flags, 1);
    if (!model_motion_blocked(flags, 1) || !b.sticky) fail++;

    return fail;
}
