
#include "subsystem4.h"

/* Buddy 4 IMU processing task.
 * Sensor communication and processing will be added incrementally.
 */
EXPORT void imu_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;) {
        /* IMU processing will be added in later steps. */
        tk_dly_tsk(10);
    }
}
