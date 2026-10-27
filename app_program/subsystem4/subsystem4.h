/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU subsystem (GY-511 / LSM303DLHC).  Student ID: 2502446.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* GY-511 / LSM303DLHC hardware configuration */
/* I2C0 configuration; the device driver handles peripheral initialisation. */
#define S4_I2C_PORT          0      
#define S4_I2C_SDA_PIN       4
#define S4_I2C_SCL_PIN       5

/* GY-511 / LSM303DLHC I2C addresses */
#define S4_ACCEL_ADDR        0x19
#define S4_MAG_ADDR          0x1E

/* IMU task configuration */
#define S4_IMU_PERIOD_MS     10
#define S4_IMU_PRIORITY      19     // provisional *NOTE

/* Detection thresholds: provisional, to be calibrated */
#define S4_SHOCK_THRESHOLD_G 2.5f     // Change to 4g // verify later how the measured acceleration is converted into g

/* Accelerometer registers */
#define S4_REG_CTRL1_A   0x20
#define S4_REG_OUT_X_L_A 0x28
#define S4_REG_OUT_X_H_A 0x29
#define S4_REG_OUT_Y_L_A 0x2A
#define S4_REG_OUT_Y_H_A 0x2B
#define S4_REG_OUT_Z_L_A 0x2C
#define S4_REG_OUT_Z_H_A 0x2D

/* Magnetometer registers */
#define S4_REG_CRA_M     0x00
#define S4_REG_MR_M      0x02
#define S4_REG_OUT_X_H_M 0x03

/* IMU event types */
typedef enum {
    S4_IMU_EVENT_HEADING,
    S4_IMU_EVENT_HUMP_START,
    S4_IMU_EVENT_HUMP_END,
    S4_IMU_EVENT_MINOR_SHOCK,
    S4_IMU_EVENT_COLLISION
} S4_IMU_EVENT_TYPE;

/* Temporary local event structure for isolated testing */
typedef struct {
    S4_IMU_EVENT_TYPE type;
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float acceleration_g;
} S4_IMU_EVENT;

/* Compatibility interface for Subsystem 3 reporting */
#define S4_SUMMARY_MAX 48
#define FLG_BATCH      0x0002
#define FLG_ALARM      0x0001

/* Original reporting monitor configuration */
#define S4_PRI_MONITOR           18
#define S4_ALARM_MS              2000
#define S4_LED_TOGGLE_REPORTS    6

IMPORT ID mbf_summary;
IMPORT ID flg_events;
IMPORT ID alm_monitor;

IMPORT UW s4_lines_received;
IMPORT void monitor_task(INT stacd, void *exinf);
IMPORT void monitor_alarm_handler(void *exinf);

/* Subsystem lifecycle functions */
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem4_start(void);
IMPORT ID s4_i2c_dev;


#endif /* SUBSYSTEM4_H */
