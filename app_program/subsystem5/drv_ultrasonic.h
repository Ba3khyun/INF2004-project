/*
 *	drv_ultrasonic.h
 *	Subsystem 5 -- HC-SR04 driver.  PRIVATE to S5.
 *	Call only from obstacle_task, the owner of the sensor (TEAM.md s.3).
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include "subsystem5.h"

#define US_MEDIAN_MAX	5U	/* most pings us_ping_median() will take */

typedef enum {
	US_OK = 0,		/* echo received, distance valid */
	US_OUT_OF_RANGE,	/* no reflection within S5_US_RANGE_MAX_CM:
				   "nothing there", still a valid reading */
	US_NO_ECHO,		/* ECHO never went high: sensor missing or
				   unpowered */
	US_STUCK		/* ECHO stuck high before triggering */
} US_STATUS;

/* Configure TRIG/ECHO pins and check the 1 MHz hardware timer */
IMPORT ER	 us_init(void);

/* Free-running microsecond counter (wraps every ~71 minutes; subtract
   two readings as unsigned values and the wrap does not matter) */
IMPORT UW	 us_now_us(void);

/* Busy-wait at least 'us' microseconds (keep it short!) */
IMPORT void	 us_delay_us(UW us);

/* One measurement.  *p_cm is S5_US_RANGE_MAX_CM unless US_OK. */
IMPORT US_STATUS us_ping(UH *p_cm);

/* Median of n pings (1..US_MEDIAN_MAX), S5_PING_GAP_MS apart */
IMPORT US_STATUS us_ping_median(UINT n, UH *p_cm);

#endif /* DRV_ULTRASONIC_H */
