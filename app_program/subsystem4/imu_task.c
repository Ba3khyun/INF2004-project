

#include "subsystem4.h"
#include <tk/device.h>
#include "../../device/include/dev_i2c.h"

#define ACCEL_ADDR       0x19
#define MAG_ADDR         0x1E

#define ACCEL_WHO_AM_I   0x0F
#define MAG_ID_A         0x0A
#define MAG_ID_B         0x0B
#define MAG_ID_C         0x0C

EXPORT void imu_task(INT stacd, void *exinf)
{
    ID iicdd;
    ER accel_er;
    ER mag_a_er;
    ER mag_b_er;
    ER mag_c_er;

    UB accel_id = 0;
    UB mag_id_a = 0;
    UB mag_id_b = 0;
    UB mag_id_c = 0;

    tm_printf((UB*)"IMU TASK STARTED\n");

    (void)stacd;
    (void)exinf;

    tm_printf((UB*)"IMU: opening I2C device iica...\n");

    iicdd = tk_opn_dev((UB*)"iica", TD_UPDATE);

    if (iicdd < E_OK) {
        tm_printf((UB*)"IMU: cannot open iica, error=%d\n",
                  (INT)iicdd);

        for (;;) {
            tk_dly_tsk(1000);
        }
    }

    tm_printf((UB*)"IMU: I2C device opened, descriptor=%d\n",
              (INT)iicdd);

    /* Read the accelerometer identity register. */
    accel_er = i2c_read_reg(
        iicdd, ACCEL_ADDR, ACCEL_WHO_AM_I, &accel_id
    );

    /* Read the three magnetometer identification bytes. */
    mag_a_er = i2c_read_reg(
        iicdd, MAG_ADDR, MAG_ID_A, &mag_id_a
    );

    mag_b_er = i2c_read_reg(
        iicdd, MAG_ADDR, MAG_ID_B, &mag_id_b
    );

    mag_c_er = i2c_read_reg(
        iicdd, MAG_ADDR, MAG_ID_C, &mag_id_c
    );

    tm_printf((UB*)"IMU: ACCEL status=%d ID=0x%02X\n",
              (INT)accel_er, (UW)accel_id);

    tm_printf((UB*)"IMU: MAG A status=%d ID=0x%02X\n",
              (INT)mag_a_er, (UW)mag_id_a);

    tm_printf((UB*)"IMU: MAG B status=%d ID=0x%02X\n",
              (INT)mag_b_er, (UW)mag_id_b);

    tm_printf((UB*)"IMU: MAG C status=%d ID=0x%02X\n",
              (INT)mag_c_er, (UW)mag_id_c);

    if (accel_er >= E_OK &&
        mag_a_er >= E_OK &&
        mag_b_er >= E_OK &&
        mag_c_er >= E_OK &&
        accel_id == 0x33 &&
        mag_id_a == 0x48 &&
        mag_id_b == 0x34 &&
        mag_id_c == 0x33) {
        tm_printf((UB*)"IMU: both sensor IDs verified!\n");
    } else {
        tm_printf((UB*)"IMU: ID check failed; inspect results above.\n");
    }

    /* Keep the task alive; sampling will be added next. */
    for (;;) {
        tk_dly_tsk(1000);
    }
}
