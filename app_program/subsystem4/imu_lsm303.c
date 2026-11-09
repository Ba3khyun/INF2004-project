/* Subsystem 4: LSM303D driver, using existing kernel I2C EXEC interface. */
#include "imu_lsm303.h"
#include "../app_debug.h"
#include "../../device/include/dev_i2c.h"

#define VECTOR_BYTES 6
#define ID_TRIES 3
#define RETRY_MS 20
static ID g_dd = -1;
static UW g_addr = LSM303D_ADDR_LOW;
static UW g_err_count = 0U;
static imu_variant_t g_variant = IMU_VAR_UNKNOWN;

static ER xfer_read(UW addr, UB reg, UB *out, SZ n)
{
    T_I2C_EXEC ex;
    SZ actual = 0;
    ER er;
    ex.sadr = addr;
    ex.snd_size = 1;
    ex.snd_data = &reg;
    ex.rcv_size = n;
    ex.rcv_data = out;
    er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &actual);
    if (er < E_OK) { g_err_count++; }
    return er;
}
static ER write_reg(UW addr, UB reg, UB value)
{
    ER er = i2c_write_reg(g_dd, addr, reg, value);
    if (er < E_OK) { g_err_count++; }
    return er;
}
EXPORT ER imu_drv_read_reg(UW addr, UW reg, UB *value)
{
    return xfer_read(addr, (UB)reg, value, 1);
}
EXPORT UW imu_drv_error_count(void) { return g_err_count; }
EXPORT imu_variant_t imu_drv_variant(void) { return g_variant; }
static int16_t le16(UB lo, UB hi)
{
    return (int16_t)(uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}
EXPORT ER imu_drv_read_accel(int16_t raw[3])
{
    UB b[VECTOR_BYTES];
    ER er = xfer_read(g_addr, (UB)(LSM303D_OUT_X_L_A | LSM303D_AUTO_INC), b, VECTOR_BYTES);
    if (er != E_OK) { return er; }
    raw[0] = le16(b[0], b[1]);
    raw[1] = le16(b[2], b[3]);
    raw[2] = le16(b[4], b[5]);
    return E_OK;
}

EXPORT ER imu_drv_read_mag(int16_t raw[3], UB *overflow)
{
    UB b[VECTOR_BYTES];
    ER er;
#if APP_DEBUG_IMU_EXTRA_I2C
    /* OPTIONAL: adds an extra status-register I2C transaction.
     * Explicitly disabled by default to protect 100 Hz sampling. */
    static UW debug_samples = 0U;
    if ((++debug_samples % 100U) == 0U) {
        UB status = 0U;
        ER status_er = xfer_read(g_addr, 0x07U, &status, 1);
        tm_printf((UB*)"[IMU-I2C] STATUS_M er=%d value=0x%02x\n",
                  (INT)status_er, (UINT)status);
    }
#endif
    /* Original six-byte magnetic burst, no debug bus traffic by default. */
    er = xfer_read(g_addr,
                   (UB)(LSM303D_OUT_X_L_M | LSM303D_AUTO_INC),
                   b, VECTOR_BYTES);
    if (er != E_OK) return er;

    raw[0] = le16(b[0], b[1]);
    raw[1] = le16(b[2], b[3]);
    raw[2] = le16(b[4], b[5]);
    *overflow = 0U;
    return E_OK;
}
EXPORT UW imu_drv_scan(void)
{
    UW addr;
    UW found = 0U;
    UW timeouts = 0U;
    tm_printf((UB*)"IMU: scanning I2C0 addresses 0x08..0x77\n");
    for (addr = 0x08U; addr <= 0x77U; addr++) {
        UB value = 0U;
        T_I2C_EXEC ex;
        SZ actual = 0;
        ER er;
        UB reg = LSM303D_WHO_AM_I;
        ex.sadr = addr;
        ex.snd_size = 1;
        ex.snd_data = &reg;
        ex.rcv_size = 1;
        ex.rcv_data = &value;
        er = tk_swri_dev(g_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &actual);
        if (er == E_OK) {
            tm_printf((UB*)"IMU: address 0x%02X ACK, reg0F=0x%02X\n", (UINT)addr, (UINT)value);
            found++;
        } else if (MERCD(er) == MERCD(E_TMOUT)) {
            timeouts++;
            if (timeouts >= 2U) {
                tm_printf((UB*)"IMU: scan stopped on repeated timeouts at 0x%02X\n", (UINT)addr);
                break;
            }
        }
    }
    tm_printf((UB*)"IMU: scan finished; %u device(s) responding\n", (UINT)found);
    return found;
}
static ER identify(void)
{
    const UW candidates[2] = { LSM303D_ADDR_LOW, LSM303D_ADDR_HIGH };
    UW i;
    INT attempt;
    ER last_err = E_SYS;
    for (i = 0U; i < 2U; i++) {
        for (attempt = 0; attempt < ID_TRIES; attempt++) {
            UB id = 0U;
            ER er = xfer_read(candidates[i], LSM303D_WHO_AM_I, &id, 1);
            if (er == E_OK) {
                tm_printf((UB*)"IMU: address 0x%02X WHO_AM_I=0x%02X\n",
                          (UINT)candidates[i], (UINT)id);
                if (id == LSM303D_WHO_ID) {
                    g_addr = candidates[i];
                    g_variant = IMU_VAR_LSM303D;
                    tm_printf((UB*)"IMU: LSM303D found at 0x%02X (ID 0x49)\n", (UINT)g_addr);
                    return E_OK;
                }
                last_err = E_NOSPT;
                break;
            }
            last_err = er;
            tk_dly_tsk(RETRY_MS);
        }
    }
    tm_printf((UB*)"IMU: LSM303D not found at 0x1D/0x1E (last error=%d)\n", last_err);
    (void)imu_drv_scan();
    return last_err;
}
EXPORT ER imu_drv_init(ID i2c_dd)
{
    static const UB registers[5] = {LSM303D_CTRL1, LSM303D_CTRL2,
                                    LSM303D_CTRL5, LSM303D_CTRL6, LSM303D_CTRL7};
    static const UB values[5] = {LSM303D_CFG_CTRL1, LSM303D_CFG_CTRL2,
                                 LSM303D_CFG_CTRL5, LSM303D_CFG_CTRL6, LSM303D_CFG_CTRL7};
    UW i;
    ER er;
    g_dd = i2c_dd;
    g_variant = IMU_VAR_UNKNOWN;
    er = identify();
    if (er != E_OK) { return er; }
    for (i = 0U; i < 5U; i++) {
        UB actual = 0U;
        er = write_reg(g_addr, registers[i], values[i]);
        if (er != E_OK) {
            tm_printf((UB*)"IMU: register 0x%02X write failed: %d\n", (UINT)registers[i], er);
            return er;
        }
        er = xfer_read(g_addr, registers[i], &actual, 1);
        if (er != E_OK) { return er; }
        if (actual != values[i]) {
            tm_printf((UB*)"IMU: register 0x%02X readback 0x%02X expected 0x%02X\n",
                      (UINT)registers[i], (UINT)actual, (UINT)values[i]);
            return E_SYS;
        }
    }
    tm_printf((UB*)"IMU: LSM303D configured accel 100Hz +-4g, mag 100Hz +-2gauss\n");
    return E_OK;
}
