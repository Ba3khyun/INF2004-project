#include "subsystem4.h"
#include "../../device/include/dev_i2c.h"
#include "../../device/i2c/i2c.h"
EXPORT void imu_task(INT stacd, void *exinf)
{
    UB accel_id = 0;
    UB mag_id = 0;
    ER err;
    UW status;
    UW abort_source;
    
    (void)stacd;
    (void)exinf;

    tm_printf((UB*)"IMU: task started\n");

    tk_dly_tsk(1000);

    /* Test accelerometer */
    tm_printf((UB*)"IMU: testing accelerometer 0x19\n");

    err = i2c_read_reg(
        s4_i2c_dev,
        S4_ACCEL_ADDR,
        0x0F,
        &accel_id
    );

    status = dev_i2c_get_error_reason(S4_I2C_PORT);
    abort_source = dev_i2c_get_abort_source(S4_I2C_PORT);

    tm_printf(
        (UB*)"ACCEL: err=%d ID=0x%02X status=0x%08X abort=0x%08X\n",
        err, accel_id, status, abort_source
    );

    tk_dly_tsk(100);

    /* Test magnetometer */
    tm_printf((UB*)"IMU: testing magnetometer 0x1E\n");

    err = i2c_read_reg(
        s4_i2c_dev,
        S4_MAG_ADDR,
        0x0A,
        &mag_id
    );

    /* Add the print HERE, after the read */
    tm_printf((UB*)"MAG ID test: err=%d value=0x%02X\n",
            err, mag_id);

    status = dev_i2c_get_error_reason(S4_I2C_PORT);
    abort_source = dev_i2c_get_abort_source(S4_I2C_PORT);

    tm_printf(
        (UB*)"MAG: err=%d ID=0x%02X status=0x%08X abort=0x%08X\n",
        err, mag_id, status, abort_source
    );

    tk_ext_tsk();
}

// #include "subsystem4.h"
// #include "../../device/include/dev_i2c.h"
// #include "../../device/i2c/i2c.h"

// EXPORT void imu_task(INT stacd, void *exinf)
// {
//     const UB addresses[] = {0x19, 0x18, 0x1E};
//     const UB registers[] = {0x0F, 0x0F, 0x0A};
//     const char *names[] = {
//         "Accelerometer (0x19)",
//         "Accelerometer (0x18)",
//         "Magnetometer (0x1E)"
//     };

//     (void)stacd;
//     (void)exinf;

//     tm_printf((UB*)"IMU: I2C address diagnostic\n");
//     tk_dly_tsk(1000);

//     for (int i = 0; i < 3; i++) {
//         UB value = 0;
//         ER err = i2c_read_reg(
//             s4_i2c_dev,
//             addresses[i],
//             registers[i],
//             &value
//         );

//         tm_printf(
//             (UB*)"IMU: %s, err=%d, value=0x%02X\n",
//             names[i], err, value
//         );
//     }

//     tk_ext_tsk();
// }



// /*
//  * imu_task.c
//  * Subsystem 4 -- temporary GY-511 I2C identification test.
//  */

// #include "subsystem4.h"
// #include "../../device/include/dev_i2c.h"

// EXPORT void imu_task(INT stacd, void *exinf)
// {
//     tm_printf((UB*)"IMU: task started\n");
//     UB accel_id = 0;
//     UB mag_id = 0;
//     ER accel_err;
//     ER mag_err;

//     (void)stacd;
//     (void)exinf;

//     tm_printf((UB*)"IMU: starting I2C identification test\n");

//     /* LSM303DLHC accelerometer WHO_AM_I register */
//     accel_err = i2c_read_reg(
//         s4_i2c_dev,
//         S4_ACCEL_ADDR,
//         0x0F,
//         &accel_id
//     );

//     /* LSM303DLHC magnetometer identification register */
//     mag_err = i2c_read_reg(
//         s4_i2c_dev,
//         S4_MAG_ADDR,
//         0x0F,
//         &mag_id
//     );

//     if (accel_err == E_OK) {
//         tm_printf((UB*)"IMU: accelerometer ID = 0x%02X\n", accel_id);
//     } else {
//         tm_printf((UB*)"IMU: accelerometer read failed, err=%d\n",
//                   accel_err);
//     }

//     if (mag_err == E_OK) {
//         tm_printf((UB*)"IMU: magnetometer ID = 0x%02X\n", mag_id);
//     } else {
//         tm_printf((UB*)"IMU: magnetometer read failed, err=%d\n",
//                   mag_err);
//     }

//     while (1) {
//         tk_dly_tsk(S4_IMU_PERIOD_MS);
//     }
// }
