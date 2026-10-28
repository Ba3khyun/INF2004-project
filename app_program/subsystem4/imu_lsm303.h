/*
 * imu_lsm303.h
 * Subsystem 4 -- driver for the GY-511 board (LSM303DLHC: accelerometer at
 * I2C 0x19, magnetometer at 0x1E) on I2C0 (GP4 SDA / GP5 SCL, Grove 3).
 *
 * Uses the kernel's I2C device driver (device/i2c, "iica").  Only the IMU
 * task may call these functions (single owner of I2C0, see TEAM.md).
 */
#ifndef IMU_LSM303_H
#define IMU_LSM303_H

#include "../app.h"
#include <stdint.h>

#define LSM_ACCEL_ADDR        0x19U
#define LSM_MAG_ADDR          0x1EU

/* accelerometer registers */
#define LSM_REG_CTRL1_A       0x20U
#define LSM_REG_CTRL4_A       0x23U
#define LSM_REG_STATUS_A      0x27U
#define LSM_REG_OUT_X_L_A     0x28U
#define LSM_REG_AUTO_INC      0x80U   /* set on the sub-address to read several bytes */
#define LSM_REG_WHOAMI_A      0x0FU   /* LSM303AGR/C/D only; the DLHC has none        */
/* magnetometer registers */
#define LSM_REG_CRA_M         0x00U
#define LSM_REG_CRB_M         0x01U
#define LSM_REG_MR_M          0x02U
#define LSM_REG_OUT_X_H_M     0x03U   /* order: X_H X_L Z_H Z_L Y_H Y_L */
#define LSM_REG_IRA_M         0x0AU   /* DLHC ID bytes: 0x48 0x34 0x33  */
#define LSM_REG_WHOAMI_M_AGR  0x4FU   /* LSM303AGR: 0x40                */

/* configuration values written at start-up */
#define LSM_CFG_CTRL1_A       0x57U   /* 100 Hz, normal power, X/Y/Z on          */
#define LSM_CFG_CTRL4_A       0x98U   /* BDU on, +-4 g, high resolution (12-bit) */
#define LSM_CFG_CRA_M         0x1CU   /* 220 Hz output rate                      */
#define LSM_CFG_CRB_M         0x20U   /* +-1.3 gauss                             */
#define LSM_CFG_MR_M          0x00U   /* continuous conversion                   */

typedef enum {
    IMU_VAR_UNKNOWN = 0,
    IMU_VAR_LSM303DLHC,
    IMU_VAR_LSM303AGR
} imu_variant_t;

/* Identify the chip and write the configuration.  E_OK, or an error code. */
IMPORT ER imu_drv_init(ID i2c_dd);
IMPORT imu_variant_t imu_drv_variant(void);

/* Raw sample readers.  raw[0..2] = X, Y, Z in sensor axes. */
IMPORT ER imu_drv_read_accel(int16_t raw[3]);
/* *overflow is set to 1 if any axis reports the sensor overflow value. */
IMPORT ER imu_drv_read_mag(int16_t raw[3], UB *overflow);

/* Single-register read (bring-up and tests). */
IMPORT ER imu_drv_read_reg(UW sadr, UW reg, UB *val);

IMPORT UW imu_drv_error_count(void);   /* failed I2C transactions since boot */

/* Bring-up diagnostic: probes every address 0x08-0x77 on I2C0 and prints the
 * ones that answer.  Returns how many answered.  Called automatically when the
 * chip does not respond.  The expected addresses are 0x19 and 0x1E. */
IMPORT UW imu_drv_scan(void);

#endif /* IMU_LSM303_H */
