# Buddy 4: LSM303D changes

Copy the four files from `INF2004-project/app_program/subsystem4/` into your equivalent directory. These are the *full replacement files* based on `INF2004-project-claude-v2.zip`.

Changed:
- `imu_lsm303.h`: LSM303D address 0x1D or 0x1E, one sensor chip, correct registers.
- `imu_lsm303.c`: WHO_AM_I(0x0F)==0x49, try both addresses, configure/readback registers, little-endian XYZ accel and mag samples, I2C address scan.
- `imu_math.h`: scaling for LSM303D +-4 g (0.122 mg/LSB) and mag +-2 gauss (0.080 mgauss/LSB).
- `subsystem4_test.c`: S4-U10 expects LSM303D.

Not changed: `imu_task.c` (it calls the unchanged driver APIs), `imu_math.c` (same algorithms), other subsystems, and shared I2C driver. **Keep the previously applied I2C transport fix**: this ZIP does not contain that file.

## Build and test

```
cmake --build --preset pico-w-test
```

Flash UF2. With serial monitor open, press RESET to see early boot diagnostics. Expected: `IMU: LSM303D found at 0x1D (ID 0x49)` or at `0x1E`, then `IMU: LSM303D configured ...`, followed by calibration.

If neither address answers, bus scan reports detected addresses. The current scan performs a write-register-then-read test, so a response depends on proper repeated-start support in the I2C driver.

## Important limitations

- These files have **not been built or flashed on the Pico**, and the first build should be treated as a verification step.
- LSM303D does not use DLHC's -4096 magnetometer overflow marker. This driver sets `overflow=0` until LSM303D-specific overflow handling is implemented; `S4-U16d` will not validate actual overflows.
- Accelerometer and magnetometer direction mappings must be checked on your installed board. `IMU_AXIS_MAP_DEFAULT` has not been validated.
- Start-up calibration test S4-U12 and physical test S4-U13/S4-U14 require a functioning connected chip and still board.
- If other team members modify low-level I2C files, coordinate changes before merging.
