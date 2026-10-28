/*
 * imu_lsm303.c
 * Subsystem 4 -- GY-511 / LSM303DLHC driver.  See imu_lsm303.h.
 */
#include "imu_lsm303.h"
#include "../../device/include/dev_i2c.h"

#define LSM_MAG_ID_A       0x48U
#define LSM_MAG_ID_B       0x34U
#define LSM_MAG_ID_C       0x33U
#define LSM_ACCEL_ID_AGR   0x33U
#define LSM_MAG_ID_AGR     0x40U
#define LSM_BYTES_PER_VEC  6
#define LSM_MAG_OVERFLOW   (-4096)

static ID            g_dd = -1;
static imu_variant_t g_variant = IMU_VAR_UNKNOWN;
static UW            g_err_count = 0U;

/* Write the register pointer, repeated start, then read n bytes. */
static ER xfer_read(UW sadr, UB reg, UB *rx, SZ n)
{
    T_I2C_EXEC ex;
    SZ  asz = 0;
    ER  er;

    ex.sadr     = sadr;
    ex.snd_size = 1;
    ex.snd_data = &reg;
    ex.rcv_size = n;
    ex.rcv_data = rx;
    er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &asz);
    if (er < E_OK) {
        g_err_count++;
        return er;
    }
    return E_OK;
}

static ER write_reg(UW sadr, UB reg, UB val)
{
    ER er = i2c_write_reg(g_dd, sadr, reg, val);
    if (er < E_OK) {
        g_err_count++;
        return er;
    }
    return E_OK;
}

EXPORT ER imu_drv_read_reg(UW sadr, UW reg, UB *val)
{
    return xfer_read(sadr, (UB)reg, val, 1);
}

EXPORT UW imu_drv_error_count(void) { return g_err_count; }
EXPORT imu_variant_t imu_drv_variant(void) { return g_variant; }

static int16_t join16(UB hi, UB lo)
{
    uint16_t u = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    return (int16_t)u;
}

EXPORT ER imu_drv_read_accel(int16_t raw[3])
{
    UB b[LSM_BYTES_PER_VEC];
    ER er = xfer_read(LSM_ACCEL_ADDR, (UB)(LSM_REG_OUT_X_L_A | LSM_REG_AUTO_INC),
                      b, LSM_BYTES_PER_VEC);
    if (er != E_OK) {
        return er;
    }
    raw[0] = join16(b[1], b[0]);   /* little-endian: low byte first */
    raw[1] = join16(b[3], b[2]);
    raw[2] = join16(b[5], b[4]);
    return E_OK;
}

EXPORT ER imu_drv_read_mag(int16_t raw[3], UB *overflow)
{
    UB b[LSM_BYTES_PER_VEC];
    ER er = xfer_read(LSM_MAG_ADDR, (UB)LSM_REG_OUT_X_H_M, b, LSM_BYTES_PER_VEC);
    if (er != E_OK) {
        return er;
    }
    raw[0] = join16(b[0], b[1]);   /* big-endian pairs, order X, Z, Y */
    raw[2] = join16(b[2], b[3]);
    raw[1] = join16(b[4], b[5]);
    *overflow = ((raw[0] == LSM_MAG_OVERFLOW) || (raw[1] == LSM_MAG_OVERFLOW) ||
                 (raw[2] == LSM_MAG_OVERFLOW)) ? 1U : 0U;
    return E_OK;
}

#define LSM_ID_TRIES     3
#define LSM_ID_RETRY_MS  20
#define LSM_SCAN_FIRST   0x08U
#define LSM_SCAN_LAST    0x77U
#define LSM_SCAN_MAX_TMO 2U      /* stop scanning if the bus hangs this many times */

static const char *err_name(ER er)
{
    if (er == E_OK)                    { return "OK"; }
    if (MERCD(er) == MERCD(E_TMOUT))   { return "TIMEOUT (bus stuck or no clock)"; }
    if (MERCD(er) == MERCD(E_IO))      { return "I/O ERROR (no ACK: nothing at this address)"; }
    if (MERCD(er) == MERCD(E_PAR))     { return "BAD PARAMETER"; }
    return "other";
}

EXPORT UW imu_drv_scan(void)
{
    UW found = 0U;
    UW hangs = 0U;
    UW addr;

    tm_printf((UB*)"IMU: scanning I2C0 (0x%02X-0x%02X)...\n", LSM_SCAN_FIRST, LSM_SCAN_LAST);
    for (addr = LSM_SCAN_FIRST; addr <= LSM_SCAN_LAST; addr++) {
        T_I2C_EXEC ex;
        SZ  asz = 0;
        UB  reg = 0U;
        UB  val = 0U;
        ER  er;

        ex.sadr     = addr;
        ex.snd_size = 1;
        ex.snd_data = &reg;
        ex.rcv_size = 1;
        ex.rcv_data = &val;
        er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &asz);
        if (er >= E_OK) {
            tm_printf((UB*)"IMU:   device answers at 0x%02X\n", addr);
            found++;
        } else if (MERCD(er) == MERCD(E_TMOUT)) {
            hangs++;
            if (hangs >= LSM_SCAN_MAX_TMO) {
                tm_printf((UB*)"IMU:   bus hangs (SDA or SCL stuck low?), scan stopped at 0x%02X\n", addr);
                break;
            }
        }
    }
    tm_printf((UB*)"IMU: scan finished, %u device(s) answered (expected 0x19 and 0x1E)\n", (UINT)found);
    return found;
}

static ER identify(void)
{
    UB id[3] = { 0U, 0U, 0U };
    UB a = 0U;
    UB m = 0U;
    ER er = E_SYS;
    INT attempt;

    for (attempt = 0; attempt < LSM_ID_TRIES; attempt++) {
        er = xfer_read(LSM_MAG_ADDR, (UB)LSM_REG_IRA_M, id, 3);
        if (er == E_OK) {
            break;
        }
        tk_dly_tsk(LSM_ID_RETRY_MS);
    }
    if (er != E_OK) {
        tm_printf((UB*)"IMU: no answer from magnetometer 0x%02X after %d tries, err=0x%X: %s\n",
                  LSM_MAG_ADDR, (INT)LSM_ID_TRIES, (UINT)er, err_name(er));
        (void)imu_drv_scan();
        return er;
    }
    if ((id[0] == LSM_MAG_ID_A) && (id[1] == LSM_MAG_ID_B) && (id[2] == LSM_MAG_ID_C)) {
        g_variant = IMU_VAR_LSM303DLHC;
        tm_printf((UB*)"IMU: LSM303DLHC found (mag ID %02X %02X %02X)\n", id[0], id[1], id[2]);
        return E_OK;
    }
    if ((xfer_read(LSM_ACCEL_ADDR, (UB)LSM_REG_WHOAMI_A, &a, 1) == E_OK) &&
        (xfer_read(LSM_MAG_ADDR, (UB)LSM_REG_WHOAMI_M_AGR, &m, 1) == E_OK) &&
        (a == LSM_ACCEL_ID_AGR) && (m == LSM_MAG_ID_AGR)) {
        g_variant = IMU_VAR_LSM303AGR;
        tm_printf((UB*)"IMU: LSM303AGR found - this driver supports the DLHC only\n");
        return E_NOSPT;
    }
    g_variant = IMU_VAR_UNKNOWN;
    tm_printf((UB*)"IMU: unknown chip (IRA..IRC = %02X %02X %02X)\n", id[0], id[1], id[2]);
    return E_NOSPT;
}

EXPORT ER imu_drv_init(ID i2c_dd)
{
    UB r1 = 0U;
    UB r4 = 0U;
    UB cra = 0U;
    ER er;

    g_dd = i2c_dd;
    g_variant = IMU_VAR_UNKNOWN;

    er = identify();
    if (er != E_OK) {
        return er;
    }

    er = write_reg(LSM_ACCEL_ADDR, (UB)LSM_REG_CTRL1_A, (UB)LSM_CFG_CTRL1_A);
    if (er == E_OK) { er = write_reg(LSM_ACCEL_ADDR, (UB)LSM_REG_CTRL4_A, (UB)LSM_CFG_CTRL4_A); }
    if (er == E_OK) { er = write_reg(LSM_MAG_ADDR, (UB)LSM_REG_CRA_M, (UB)LSM_CFG_CRA_M); }
    if (er == E_OK) { er = write_reg(LSM_MAG_ADDR, (UB)LSM_REG_CRB_M, (UB)LSM_CFG_CRB_M); }
    if (er == E_OK) { er = write_reg(LSM_MAG_ADDR, (UB)LSM_REG_MR_M,  (UB)LSM_CFG_MR_M); }
    if (er != E_OK) {
        tm_printf((UB*)"IMU: configuration write failed (err=%d)\n", er);
        return er;
    }

    /* Read back to prove the writes really reached the chip. */
    er = xfer_read(LSM_ACCEL_ADDR, (UB)LSM_REG_CTRL1_A, &r1, 1);
    if (er == E_OK) { er = xfer_read(LSM_ACCEL_ADDR, (UB)LSM_REG_CTRL4_A, &r4, 1); }
    if (er == E_OK) { er = xfer_read(LSM_MAG_ADDR, (UB)LSM_REG_CRA_M, &cra, 1); }
    if (er != E_OK) {
        return er;
    }
    if ((r1 != LSM_CFG_CTRL1_A) || (r4 != LSM_CFG_CTRL4_A) || (cra != LSM_CFG_CRA_M)) {
        tm_printf((UB*)"IMU: readback mismatch CTRL1_A=%02X CTRL4_A=%02X CRA_M=%02X\n", r1, r4, cra);
        return E_SYS;
    }
    tm_printf((UB*)"IMU: configured (100 Hz accel +-4 g, 220 Hz mag)\n");
    return E_OK;
}
