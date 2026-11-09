/* Phase 4A: observe-only Safety Handler. NO shared flag writes or motor calls.
 * This is NOT a functioning physical emergency stop! Do not energize motors.
 * The priority-5 task reads the existing flg_safety object. Its logical
 * latch is independent from the actual SAFETY_ESTOP_LATCHED bit.
 */
#include "safety_handler.h"
#include "../app_debug.h"
#include "safety_core.h"
#include "safety_estop_bridge.h"

/* Phase 4D integration switch, deliberately OFF for all shipped builds.
 * Do NOT enable without Buddy 2/Buddy 5 safety review and a test plan.
 * Active path asserts ESTOP ONLY; it never clears it. */
#define SH_ENABLE_SHARED_ESTOP_ASSERT 0
#if APP_TEST && SH_ENABLE_SHARED_ESTOP_ASSERT
#error "Do not enable live ESTOP assertion while subsystem self-tests manipulate safety flags"
#endif

#define SH_OBSERVER_PERIOD_MS 50
#define SH_DEBUG_PRINT APP_DEBUG_SAFETY

static safety_core_t g_safety;
#if SH_ENABLE_SHARED_ESTOP_ASSERT
static seb_state_t g_estop_bridge;
#endif
static UW g_last_latched;
static UW g_last_reasons;

UW safety_handler_would_latch(void)
{
    return __atomic_load_n(&g_last_latched, __ATOMIC_ACQUIRE);
}

UW safety_handler_reasons(void)
{
    return __atomic_load_n(&g_last_reasons, __ATOMIC_ACQUIRE);
}

ER safety_handler_init(void)
{
    safety_core_init(&g_safety);
#if SH_ENABLE_SHARED_ESTOP_ASSERT
    seb_init(&g_estop_bridge);
#endif
    __atomic_store_n(&g_last_latched, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&g_last_reasons, 0u, __ATOMIC_RELEASE);
    return E_OK;
}

static void safety_handler_task(INT stacd, void *exinf)
{
#if SH_DEBUG_PRINT
    UW previous = 0u;
#endif
    UW printed_error = 0u;
    (void)stacd;
    (void)exinf;

    for (;;) {
        T_RFLG flag;
        ER er = tk_ref_flg(flg_safety, &flag);
#if SH_ENABLE_SHARED_ESTOP_ASSERT
        /* Live assert-only integration. No clear/reset/motor commands.
         * On an unreadable flag, assert ESTOP conservatively.
         * Check tk_set_flg return value; repeated ticks retry. */
        if (seb_step(&g_estop_bridge,
                     er == E_OK ? (uint32_t)flag.flgptn : 0u,
                     er == E_OK)) {
            ER set_er = tk_set_flg(flg_safety, SAFETY_ESTOP_LATCHED);
            if (set_er < E_OK && !printed_error) {
                tm_printf((UB*)"[SAFETY] ESTOP assert FAILED: %d\n", (INT)set_er);
                printed_error = 1u;
            }
        }
#endif
        if (er == E_OK) {
            safety_core_observe(&g_safety, (uint32_t)flag.flgptn);
            __atomic_store_n(&g_last_latched, g_safety.latched, __ATOMIC_RELEASE);
            __atomic_store_n(&g_last_reasons, g_safety.pending_reasons,
                             __ATOMIC_RELEASE);
#if SH_DEBUG_PRINT
            if (g_safety.latched && !previous) {
                tm_printf((UB*)"[SAFETY] software latch=1 reasons=0x%x "
                          "(OBSERVE ONLY; shared ESTOP unchanged)\n",
                          (UINT)g_safety.pending_reasons);
            }
#endif
#if SH_DEBUG_PRINT
            previous = (UW)g_safety.latched;
#endif
        } else {
            /* Conservative diagnostic state if the shared flag cannot be read.
             * Do not claim the physical motor is stopped. */
            __atomic_store_n(&g_last_latched, 1u, __ATOMIC_RELEASE);
            if (!printed_error) {
                tm_printf((UB*)"[SAFETY] tk_ref_flg failed: %d "
                          "(OBSERVER FAULT; motor safety NOT assured)\n", (INT)er);
                printed_error = 1u;
            }
        }
        tk_dly_tsk(SH_OBSERVER_PERIOD_MS);
    }
}

ER safety_handler_start(void)
{
    ID id = app_start_task(safety_handler_task, PRI_SAFETY, 0,
                           "safety_handler_task");
    return (id < E_OK) ? (ER)id : E_OK;
}
