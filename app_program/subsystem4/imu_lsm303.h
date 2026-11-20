/* Subsystem 4: LSM303D 3-axis accelerometer + magnetometer over I2C0. */
#ifndef IMU_LSM303_H
#define IMU_LSM303_H
#include "../app.h"
#include <stdint.h>

#define LSM303D_ADDR_LOW     0x1DU
#define LSM303D_ADDR_HIGH    0x1EU
#define LSM303D_WHO_AM_I     0x0FU
#define LSM303D_WHO_ID      0x49U
#define LSM303D_CTRL1       0x20U
#define LSM303D_CTRL2       0x21U
#define LSM303D_CTRL5       0x24U
#define LSM303D_CTRL6       0x25U
#define LSM303D_CTRL7       0x26U
#define LSM303D_OUT_X_L_M   0x08U
#define LSM303D_OUT_X_L_A   0x28U
#define LSM303D_AUTO_INC    0x80U

/* Accel: 100 Hz, block update enabled, XYZ on; +-4g. */
#define LSM303D_CFG_CTRL1   0x6FU
#define LSM303D_CFG_CTRL2   0x08U
/* Mag: high resolution, 100 Hz, +-2 gauss, continuous conversion. */
#define LSM303D_CFG_CTRL5   0x74U // Changed to enable high-resolution mode for the magnetometer (from 0x78U to 0x74U)
#define LSM303D_CFG_CTRL6   0x00U
#define LSM303D_CFG_CTRL7   0x00U

typedef enum { IMU_VAR_UNKNOWN = 0, IMU_VAR_LSM303D } imu_variant_t;
IMPORT ER imu_drv_init(ID i2c_dd);
IMPORT imu_variant_t imu_drv_variant(void);
IMPORT ER imu_drv_read_accel(int16_t raw[3]);
IMPORT ER imu_drv_read_mag(int16_t raw[3], UB *overflow);
IMPORT ER imu_drv_read_reg(UW sadr, UW reg, UB *val);
IMPORT UW imu_drv_error_count(void);
IMPORT UW imu_drv_scan(void);
#endif
