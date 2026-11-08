/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */
#include "app_data.h"		/* shared bit-level data formats (all buddies) */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* IS-06 task priorities (Design Review section 3; lower = higher).
   CNF_MAX_TSKPRI is 48 so that 40 is a legal priority. */
#define PRI_SAFETY		5	/* Emergency/Safety Event Handling (B4/B5) */
#define PRI_MOTION		10	/* Motion Control (B2) */
#define PRI_VEHICLE_CTRL	12	/* Vehicle Controller (B4) */
#define PRI_LINE_FOLLOW		15	/* Line Following (B3) */
#define PRI_IMU			18	/* IMU Processing (B4) */
#define PRI_OBSTACLE		20	/* Obstacle Scanning & Avoidance (B5) */
#define PRI_BARCODE		22	/* Barcode Decoding (B3) */
#define PRI_WIFI		40	/* Wi-Fi Telemetry (B1) = cyw43 service task */


/* ---------------------------------------------------------------------
 *  Shared kernel objects created by app_shared.c BEFORE any subsystem's
 *  _init().  INTERIM: the Vehicle Controller (B4) and Safety Handler
 *  (B4/B5) code does not exist yet; when it lands, the owner moves the
 *  creation into its own subsystem and keeps these names.
 *    remote_cmd_q  message buffer, 4 x RCMD_MSG_LEN (app_data.h)
 *                  writer: Wi-Fi (B1)   reader: Vehicle Controller (B4)
 *    flg_safety    event flag, SAFETY_* bits (app_data.h)
 *                  REQ bits: set by requester, cleared by Safety
 *                  SAFETY_ESTOP_LATCHED: set/cleared by Safety ONLY
 *    flg_fault     event flag, FLT_* bits (app_data.h); each bit is
 *                  set/cleared only by its owner with tk_set_flg/tk_clr_flg
 * --------------------------------------------------------------------- */
#define REMOTE_CMD_Q_DEPTH	4
IMPORT ID	remote_cmd_q;
IMPORT ID	flg_safety;
IMPORT ID	flg_fault;
IMPORT ER	app_shared_init(void);

/* Bytes a micro T-Kernel message buffer uses per message: a 4-byte INT
   length header plus the body rounded up to 4 (kernel/tkernel/messagebuf.h) */
#define APP_MBF_SLOT(msgsz)	(4 + (((msgsz) + 3) & ~3))

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
