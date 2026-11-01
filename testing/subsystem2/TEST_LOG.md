# Subsystem 2 — test log

**Subsystem:** Motion Control (Buddy 2) · also hosts the example Producer · **Student ID:** <student ID> · **Code:** `app_program/subsystem2/` (`drv_motion.c`, `task_motion.c`)

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 (example) | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size (example) | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | `motion_cmd_q` geometry: 1-byte commands, 4 deep, empty | motion_cmd_q | auto | `tk_ref_mbf` | PASS |
| S2-U04 | `motion_task` at priority 10, sleeps between ticks | task table | auto | sample state 3 ms after each tick, 20× | ≥ 18 WAIT |
| S2-U05 | `motion_cmd_post` rejects unknown codes | motion_cmd_q | auto | `'Z'`, `0`, `'f'` | all `E_PAR` |
| S2-U06 | Motor PWM: GP8–11 on slices 4/5, enabled, divider 1.0, TOP 6249 (20 kHz) | motor driver | auto | read PWM and IO_BANK0 registers | PASS |
| S2-U07 | Idle: all four motor compare values 0 for 200 ms | brake at idle | auto | sample CC registers | all 0 |
| S2-U08 | Encoders GP6/GP7: SIO input, pull-up on, pull-down off, rising-edge IRQ enabled on core 1 | encoder driver | auto | pad, funcsel, `PROC1_INTE0` | PASS |
| S2-U09 | Safety ESTOP latched: F, L, R refused; PWM stays 0; no `FLT_MOTOR` | flg_safety ESTOP, NFR 50 ms stop | auto | test plays Safety: sets `SAFETY_ESTOP_LATCHED` | CC 0 for 450 ms |
| S2-U10 | Local stop `X`: forward refused, PWM 0, `X` is not a fault | local stop | auto | | PASS |
| S2-U11 | Clear `C`: back to IDLE, still braked, `FLT_MOTOR` clear | FLT_MOTOR ownership | auto | | PASS |
| S2-U12 | `motion_task` posts `TQ_FIELD_SPEED` at 10 Hz to Buddy 1, none dropped | IF-B1-01 | auto | `wifi_qstats()` received over 2 s | 18–22, 0 drops |
| S2-U13 | Control tick really runs at 100 Hz, no missed tick; also prints how long `tk_dly_tsk(10)` lasts | 10 ms control period | auto | `motion_tick_count` over 2 s vs 1 MHz timer | 100 ± 2 % per s, 0 missed |
| S2-M01 | **Wheels lifted.** Forward 0.5 rev: PWM on, both encoders count, stops by itself | motion | auto, `make … TEST=1` with `-DS2_TEST_MOTORS=1` | | PASS, ≥ 300 pulses per wheel |
| S2-M02 | **Wheels lifted.** ESTOP latched during a move → PWM 0 | NFR 50 ms emergency stop | auto, as M01 | time from latch to all CC = 0 | ≤ 50 ms, stays 0 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Defects found on hardware

| # | Defect | Fix | Evidence |
|---|---|---|---|
| D1 | The "10 ms" control loop waited with `tk_rcv_mbf(motion_cmd_q, …, next_update − now)`. `kernel/tkernel/timer.c` adds one tick (`TIMER_PERIOD`, 10 ms) to every relative timeout, so each wait lasted ~20 ms and the loop ran at ~50 Hz. ESTOP polling, encoder checks and target detection all ran at half the intended rate. | A 10 ms cyclic handler (`cyc_motion`, core 1) wakes `motion_task`; commands are drained with `TMO_POL` each tick; queued wake-ups are counted as missed ticks. | Cause found by code reading after S2-U04 failed in run 1; S2-U13 measures it in run 2. |

| D2 | After a **cold power-up** the RP2040 TIMER (`TIMERAWL`, 1 MHz) ran about 43× slow, so `tk_dly_tsk(10)` measured 468 µs and S2-U04/U13 failed. Cause (hypothesis, register not read): `startup_clock()` never started the watchdog tick generator; after a BOOTSEL flash the ROM bootloader leaves it correct. | Kernel: `cpu_clock.c` writes `WDOG_TICK = ENABLE | 12`. Test S1-U25 checks it. | Run 3 (`logs/2026-11-01_target_run3_coldboot_slow_timer.txt`); **fix unverified on the board**. |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S2-U01…U13, run 3, first cold power-up | uncommitted working tree | Kiro (AI assistant), board connected by Jason | 11 PASS / 2 FAIL (U04, U13) | `logs/2026-11-01_target_run3_coldboot_slow_timer.txt` | Both failures follow from defect D2 (slow TIMER: tick measured at 4267/s, `tk_dly_tsk(10)` = 468 µs). Not a verdict on the 10 ms cyclic tick: **S2-U13 must be repeated after D2 is fixed and verified.** U06–U12 passed (PWM, encoders, ESTOP refusal, X/C, speed telemetry 20 reports/2 s, queue delay max 36 ms). Motors never driven. |
| 2026-10-30 | S2-U01…U12, run 1 (Pico W, `NET=lwip MQTT=1 TEST=1`, sha256 `5b0737b3…`) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant), board connected by Jason | 11 PASS / 1 FAIL | `logs/2026-10-30_target_run1.txt` | First run on hardware. Motors never driven (`S2_TEST_MOTORS=0`). S2-U04 FAIL was a test-design fault (state sampled on the same tick that wakes `motion_task`); investigating it found D1. |
| 2026-11-01 | run 4, fixed-image cold-power capture | uncommitted working tree | Kiro (AI assistant), board connected by Jason | INCOMPLETE / UNVERIFIED for S2-U04, S2-U13, S2-M02, S2-M03 | `logs/2026-11-01_target_run4_coldboot_fixed.txt` | Capture ends at 35 s before those result lines; no pass is claimed. |
| 2026-11-01 | run 5, normal replug motor test (user-reported evidence) | uncommitted working tree | Jason | S2-M02 FAIL; S2-M03 and the second failed test UNVERIFIED | no raw capture supplied | User reported PWM remained `0x04E204E2` for the watched 100 ms after ESTOP and the move completed at 515/1000 and 500/1000 revolutions. This row records the report only. |
| 2026-11-01 | motor-test firmware build | uncommitted working tree | Copilot SDK assistant | PASS, exit 0 | `logs/2026-11-01_motor_image_build.txt` | Built `MOTORS_mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.uf2` with `S2_TEST_MOTORS=1`; not flashed or run by this session. |
| 2026-11-01 | S2-U01…U13, S2-M01…M03, run 6, normal replug with wheels lifted | uncommitted working tree | Copilot SDK assistant, board replugged by Jason | PASS 16/16, 0 failed | `logs/2026-11-01_target_run6_normal_replug_motor.txt` | Console shows S2-U04 PASS, S2-U13 PASS, S2-M03 PASS (`11` ticks during the reported 100 ms sleep, `missed 0`), S2-U12 PASS (10 Hz speed telemetry, no drops), S2-M01 PASS, and S2-M02 PASS: `ESTOP -> PWM 0: 19926 us`, then PWM stayed 0. Note: this capture reports the M03 branch as `sleep`, not the intended core-0 spin branch, so spin-starvation evidence remains incomplete despite the test PASS. |
| 2026-11-01 | S2-M03 corrected-image rerun, normal replug with wheels lifted | uncommitted working tree | Copilot SDK assistant, board replugged by Jason | S2-U04/U12/U13/M01/M02/M03 PASS; M03 branch evidence still incomplete | `logs/2026-11-01_target_run7_normal_replug_m03_fix.txt` | Updated image SHA256 starts `A09294BE`; console still reports `100 ms sleep on core 0`, not `spin`, although the test condition passes with 11 ticks and 0 missed. The affinity/processor-number test-harness issue needs a dedicated pinned core-0 spinner before M03 can be marked fully verified. |
