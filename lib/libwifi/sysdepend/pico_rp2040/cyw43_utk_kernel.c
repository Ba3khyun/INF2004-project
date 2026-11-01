/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>

IMPORT void cyw43_utk_task(INT stacd, void *exinf);

/* IS-06: this task IS the Wi-Fi Telemetry Task (Buddy 1).  It runs the
 * radio, lwIP, MQTT and -- with MQTT=1 -- the application state machine via
 * wifi_app_poll(), so it takes the Wi-Fi Telemetry priority from the team's
 * task table: 40, the lowest application priority, below Safety (5), Motion
 * Control (10), Vehicle Controller (12), Line Following (15), IMU (18),
 * Obstacle Scanning (20) and Barcode Decoding (22).  Network work therefore
 * only runs when no control task is ready on core 0.  It was 4 in the
 * template, i.e. above every application task including Safety.
 * Needs CNF_MAX_TSKPRI > 40 (config/config.h).  Must equal S1_PRI_WIFI in
 * app_program/subsystem1/subsystem1.h; S1-U05 checks it on target. */
#ifndef CYW43_TASK_PRIORITY
#define CYW43_TASK_PRIORITY  40
#endif
#if defined(CNF_MAX_TSKPRI) && (CNF_MAX_TSKPRI <= CYW43_TASK_PRIORITY)
#error "CNF_MAX_TSKPRI must be greater than CYW43_TASK_PRIORITY"
#endif
#if TM_NET_LWIP
/* lwIP's own call chains (tcp_input/dhcp/etharp) run on this same task and
 * go deeper than the bare radio poll loop did. */
#define CYW43_TASK_STACK     (8 * 1024)
#else
#define CYW43_TASK_STACK     (6 * 1024)
#endif

EXPORT ER cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

    /* Pinned to processor 1 (core 0) for the same reason usb_console.c pins
     * its service task: the CYW43439's PIO/DMA and pin ownership, and the
     * lwIP state serviced alongside it, must not migrate between cores. On a
     * single-core build there is only one processor, so the affinity request
     * is dropped rather than being passed an ID the kernel does not define. */
#if TK_SUPPORT_SMP
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
    ctsk.assprc = 0;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

    task = tk_cre_tsk(&ctsk);
    if(task < E_OK) return task;
    return tk_sta_tsk(task, 0);
}
