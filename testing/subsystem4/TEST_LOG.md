# Subsystem 4 — test log

**Subsystem:** IMU (GY-511 / LSM303DLHC)  ·  **Student ID:** 2502446  ·  **Code:** `app_program/subsystem4/`, `app_program/app_data.h`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| H-01 … H-33 | imu_math.c and the packed formats in app_data.h (unit conversion, pitch/roll/heading, BAM8/offset-binary, calibration, shock, hump height, round-trip of every field, accuracy of the built-in sqrt/sin/atan2 against libm) | NFR4.4, NFR4.5, FR4.4, Minimum-Bit requirement | host auto | `cd testing/subsystem4/host && make run` | `[HOST] SUMMARY 33 passed, 0 failed` |
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 (example link) | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U10 | chip identified as LSM303DLHC | hardware bring-up | auto | flash a `pico-w-test` build, board still | PASS |
| S4-U12 | calibration finishes, task reaches MONITORING within 4.5 s | NFR4.5 | auto | as above | PASS |
| S4-U13 | stationary \|a\| between 0.9 g and 1.1 g | accel range/scale | auto | board still | PASS |
| S4-U14 | magnetic field 0.15–1.2 gauss | magnetometer scale | auto | board still | PASS |
| S4-U15 / U15b | snapshot valid + MONITORING; no IMU fault bits | Minimum-Bit snapshot, NFR4.7 | auto | as above | PASS |
| S4-U16a–d | ~300 cycles in 3 s; worst cycle < 5 ms; worst period < 20 ms; no mag overflow | NFR4.1, NFR4.2 | auto | as above | PASS (also read the `[INFO] S4 timing` line) |
| S4-U17 | imu_evt_q holds exactly 8 messages | queue depth 8, kernel overhead | auto | as above | PASS; `[INFO]` shows accepted count |
| S4-U18a–c | packed words and BAM8/offset maths correct on the Pico | Minimum-Bit requirement | auto | as above | PASS |
| S4-U19 | `tk_set_flg` ORs, `tk_clr_flg(~bit)` clears only that bit | shared fault mask | auto | as above | PASS |
| S4-U20 | no imu events dropped | queue sizing | auto | as above | PASS |
| INT-02 | summary lines flow from subsystem 3 to subsystem 4 | IF-03 (example) | auto | keep until Subsystem 3 is replaced | PASS |
| S4-M01 | start-up banner | bring-up | manual | Flash normal image, open console | `IMU: LSM303DLHC found (mag ID 48 34 33)` then `configured`, then `calibrated` within ~2.5 s |
| S4-M02 | sensor axes vs vehicle axes | axis map | manual | Car flat: note which axis reads ~1 g (that is Z). Lift the nose ~20°: the axis that changes is forward. Edit `IMU_AXIS_MAP_DEFAULT` if needed, record the final map here | pitch rises when the nose goes up; roll rises when the left side goes up |
| S4-M03 | pitch/roll accuracy | A4.1 (step 1) | manual | Tilt on blocks/protractor at −10°, 0°, +10°, +20°; read `pitch_x10` on the console | within ±1.0° of the protractor |
| S4-M04 | heading at N, E, S, W, motors off | A4.3 | manual | Car flat, point at each direction with a phone compass | within ±10° (BAM step is 1.4°) |
| S4-M05 | heading with motors running | A4.2 | manual | Wheels lifted, motors at the measurement duty; compare with motors off at 8 compass points | shift ≤ 5° (otherwise use the magnetometer only as a disagreement flag) |
| S4-M06 | shock thresholds | A4.7 | manual | Log the peak `\|a\|` for a light tap, a table drop, a bump into a wall at cruise speed | tap/driving < 1.5 g; wall bump ≥ 2.5 g in 10 of 10 (`CRITICAL shock` printed) |
| S4-M07 | I2C failure | NFR4.7 | manual | Unplug SDA while running | within 3 cycles: `-> FAULT (fault bit 0x100)`; IMU_FAULT event posted |
| S4-M08 | hump height | FR4.4, NFR4.6, A4.1 | manual — **needs Buddy 2's `motion_get_distance_mm()`** | Drive 3 known humps ≥ 5 passes each | within ±5 mm or ±20 % in ≥ 8 of 10 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **host auto** = run on the PC;
**manual** = a person observes the result (console, protractor, phone compass…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | H-01…H-33 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
