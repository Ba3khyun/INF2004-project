# Subsystem 1 — test log

**Subsystem:** Wi-Fi Communication, Command and Telemetry (Buddy 1) · also hosts the template LED example  ·  **Student ID:** 2500915 (Lai Jia Bao)  ·  **Code:** `app_program/subsystem1/`, `app_program/app_data.h`, `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results, and
`docs/buddy1_wifi_telemetry.md` for the design these tests check.

Three kinds of test:

* **host** — PC unit tests of the pure-C modules (`testing/subsystem1/host/run_host_tests.sh`). Network, kernel and clock are faked, so they prove the logic, not Pico W timing.
* **auto** — on-target `TEST_CHECK`s in `subsystem1_test.c` (`make TEST=1` or `make NET=lwip MQTT=1 TEST=1`).
* **manual** — needs the robot, an access point, a broker and/or a logic analyser.

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | LED cyclic handler is running (template) | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (template) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms (template) | | manual | Flash normal image, scope GP19 | 2 Hz, 50 % duty |
| S1-U04 | telemetry_q geometry: 4-byte messages, 128-byte buffer, empty | IF-B1-01 | auto | `tk_ref_mbf` | PASS |
| S1-U05 | Wi-Fi Telemetry Task runs at priority 40 | task table, NFR1.2 | auto | priority read by `tk_ref_tsk(TSK_SELF)` inside `wifi_app_poll` | PASS |
| S1-U06 | Wi-Fi task sleeps between polls (no busy loop) | NFR1.2 | auto | sample task state 20× | ≥ 18 of 20 WAIT |
| S1-U07 | Telemetry golden frame on target = PC encoder | data repr. | auto | encode fixed values | `45 34 34 12 FB 19 28 A5 00` |
| S1-U08 | Command golden frame + CRC-8 check value on target | data repr. | auto | | `05 07 54`, CRC 0xF4 |
| S1-U09 | Drop-oldest, non-blocking post into a 16-slot buffer | A1.3 | auto | 20 posts into a private buffer with telemetry_q geometry | 4 dropped, newest 16 kept in order |
| S1-U10 | `telemetry_post` rejects invalid field/value | data repr. | auto | | all `E_PAR` |
| S1-U11 | telemetry_q residency ≤ 50 ms, idle system | NFR1.1 | auto | 11 posts, read `wifi_qstats()` | max ≤ 50 ms |
| S1-U12 | Valid abort → `flg_safety` REMOTE_ABORT + `remote_cmd_q` | IF-B1-02/03, A1.4 | auto | second SM, real kernel objects | PASS |
| S1-U13 | Duplicate abort ignored | A1.4 | auto | | nothing posted |
| S1-U14 | Malformed payload ignored | A1.4 | auto | 2-byte payload | nothing posted |
| S1-U15 | Unknown command ignored | A1.4 | auto | type 0, valid CRC | nothing posted |
| S1-U16 | Pause → `remote_cmd_q` only | A1.4 | auto | | PAUSE, no Safety flag |
| S1-U17 | Resume rejected while ESTOP latched | A1.4 | auto | test plays Safety, sets latch | nothing posted |
| S1-U18 | Resume accepted after local ESTOP reset | A1.4 | auto | test clears latch | RESUME posted |
| S1-U19 | Resume with old sequence rejected | A1.4 | auto | | nothing posted |
| S1-U20 | Outbound backlog ≤ 16 frames | A1.3 | auto | `wifi_stats()` | backlog_max ≤ 16 |
| S1-U21 | Concurrent fault-bit updates never lose a bit | R1 | auto | 2 tasks × 5000 set/clear of different bits in `flg_fault` | 0 errors |
| S1-U22 | `flg_fault` holds only defined bits; B1 never stores ESTOP | fault ownership | auto | | PASS |
| S1-U23 | **A (software proxy)**: 10 ms loop at Line Following's priority (15) on core 0, the Wi-Fi task's core, while Wi-Fi joins/reconnects | A1.1, NFR1.2 | auto | cyclic handler wakes a probe task; 1 MHz timer stamps; 1000 cycles | jitter ≤ 5 ms, 0 missed |
| S1-U24 | Radio up, state machine running, frames built at 10 Hz, backlog bounded; prints radio/link/IP and SM statistics | A1.2, A1.3 | auto | `cyw43_utk_get_status`, `wifi_stats()` | PASS |
| S1-U25 | The 1 MHz TIMER agrees with the kernel clock (found broken after a power cycle) | all timing | auto | `tk_dly_tsk(100)` ×3 measured on `TIMERAWL`; read `WDOG_TICK` | 100–125 ms each; `WDOG_TICK` running with 12 cycles (`…20c`) |
| S1-M06 | **Controlled link-loss trials**: switch the hotspot off/on 10×, record `LINK_DOWN` and `RECOVERED` times | A1.2, NFR1.3 | manual | console log + broker log | detect ≤ 2 s, recover without reset, 10/10 |

Changes on 2026-10-30 after the first on-target run (Buddy 2's motion code is now in the same image):
* S1-U04 no longer requires `telemetry_q` to be empty: `motion_task` posts speed from boot, before the Wi-Fi task's first poll.
* S1-U06 counts `wifi_app_poll()` calls over 2 s instead of sampling the task state. Sampling after `tk_dly_tsk` lands on the same tick that wakes the Wi-Fi task.
* S1-U11 uses `TQ_FIELD_OBSTACLE` (no producer in this build); `TQ_FIELD_SPEED` now belongs to `motion_task`. It is judged only while in SHARE_TELEMETRY (NFR1.1 says "while connected") and prints NOT-RUN otherwise.
* S1-U21's helper toggles `FLT_IMU_I2C` instead of `FLT_MOTOR`, which `motion_task` now owns.
| S1-H01…H17 | Binary frame correctness: round trip, min, max, n/a, invalid values, signed, golden LE vector, reserved bits, version, lengths, exhaustive validity, queue messages, CRC, command vectors (test E) | data repr. | host | `run_host_tests.sh` | all PASS |
| S1-H11 | Telemetry sequence wraps 255 → 0 | data repr. | host | | PASS |
| S1-H20…H34 | Remote command safety: valid/duplicate abort, malformed, unknown, version/reserved, invalid sequence, pause, resume vs ESTOP, reconnect while latched, abort with link loss, abort in RECONNECTING, queue full, report-once fault, ESTOP read-only (test D, logic) | A1.4 | host | | all PASS |
| S1-H40…H51 | State machine: start-up path, 10 Hz, silent TCP loss ≤ 2 s (10 trials), Wi-Fi loss, MQTT loss, bounded back-off, recovery without reset, 30 s outage backlog, 5 min soak, publish busy, join/MQTT timeouts, overrun (tests B/C, logic only) | NFR1.1/1.3, A1.2/1.3 | host | | all PASS |
| S1-H52 | No false link-down when the MQTT layer stamps TCP progress after the poll's `now` (tick or preemption in between); regression for defect C13 | NFR1.3, A1.2 | host | publishes stamp progress 10 ms ahead of `now`, acks arrive one poll later, 3 s | stays in SHARE_TELEMETRY, 0 LINK_DOWN events |
| S1-H60 | JSON vs binary payload sizes (test F) | | host | printed measurements | binary ≥ 10× smaller |
| S1-M01 | **A. Wi-Fi interference**: Motion Control / Line Following jitter with normal publishing, AP off, AP restored | A1.1, NFR1.2 | manual | GPIO toggle at the start of each 10 ms cycle of both tasks, logic analyser, 60 s per phase | worst jitter ≤ 5 ms, 0 missed cycles |
| S1-M02 | **B. Link-loss detection**, 10 trials | A1.2, NFR1.3 | manual | cut AP power; log `wifi: ... LINK_DOWN ... ms after last TCP ack` and `RECOVERED`; broker-side `decode --stdin` | detection ≤ 2 s and recovery without reset, 10/10 |
| S1-M03 | **C. Telemetry queue**, 10 Hz for 5 min with a 30 s outage | A1.3, NFR1.1 | manual | `wifi: stats/queue` lines every 10 s + broker log | queue bounded, drop-oldest, ≤ 50 ms residency while connected, no growth |
| S1-M04 | **D. Remote command safety** on the robot with motors powered | A1.4 | manual | `tools/wifi_telemetry_tool.py cmd ...` + `mosquitto_pub -f`; needs Safety + VC + Motion code | no unsafe movement; correct routing |
| S1-M05 | **F. RAM/size on target**: queue free bytes, static RAM | | manual | `arm-none-eabi-nm -S`, `tk_ref_mbf` | matches docs |
| INT-B1-01…05 | **G. Integration** (Wi-Fi + VC, + Safety, + producers, outage with motors/sensors active, abort with all tasks active) | IF-B1-01…03 | manual | see `testing/integration/TEST_LOG.md` | blocked until the other subsystems exist |

*Method:* **auto** = `TEST_CHECK` on target; **host** = PC unit test; **manual** = observed on hardware.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-29 | S1-H01…H60 (48 checks) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant) | PASS 48/0 | `logs/2026-10-29_host_unit.txt` | gcc 13.3 on WSL Ubuntu 24.04, ASan+UBSan, `-Werror`; `gcc -fanalyzer` clean; `wifi_telemetry_tool.py selftest` PASS. Logic only: fake network/kernel/clock. Re-run after committing to record a commit hash. |
| 2026-10-29 | build matrix (6 profiles) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant) | PASS (0 warnings) | `logs/2026-10-29_build_matrix.txt` | compile/link only; nothing was flashed |
| 2026-10-29 | S1-H52 (before fix) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant) | FAIL (expected) | `logs/2026-10-29_r2_revalidation.txt` §1 | reproduces defect C13: false SOCK_TIMEOUT when progress is stamped after the poll's `now` |
| 2026-10-29 | S1-H01…H60 incl. H52 (49 checks), run 2 | uncommitted working tree on `2eb0a68` | Kiro (AI assistant) | PASS 49/0 | `logs/2026-10-29_r2_revalidation.txt` §2 | after the C13 fix in `wifi_sm.c`; `-fanalyzer` clean; only summary lines captured |
| 2026-10-29 | build matrix, run 2 | uncommitted working tree on `2eb0a68` | Kiro (AI assistant) | PASS 5/5 profiles, 0 warnings; `CONSOLE=uart WIFI=none` not confirmed | `logs/2026-10-29_r2_revalidation.txt` §3 | compile/link only; nothing flashed |
| 2026-10-30 | S1-U01…U24, run 1 (Pico W, `NET=lwip MQTT=1 TEST=1`, image sha256 `5b0737b3…`) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant), board connected by Jason | 20 PASS / 3 FAIL | `logs/2026-10-30_target_run1.txt` | **First run on hardware.** PASS: U05 (task pri 40), U07/U08 golden frames on target, U09 drop-oldest, U10, U12–U19 routing with real kernel objects, U20, U21 (0 lost bits), U22, U23 (jitter 1009 µs, 0 missed in 1000 cycles while Wi-Fi was joining), U24. FAIL U04/U06/U11: test-design faults caused by Buddy 2 sharing the image and by tick-aligned sampling (see changes above); not firmware faults. AP "hide on bush" was never found (`SSID not found`), so nothing was published: no MQTT result exists yet. Reconnect cycle seen on target: CONNECTING_WIFI → RECONNECTING → CONNECTING_WIFI with back-off, no reset, backlog held at 16 with drop-oldest. |
| 2026-11-01 | S1-U01…U24, run 2 (Pico W on the hotspot; broker = Mosquitto 2.1.2 on the PC, port 1884; image sha256 `af09ad75…`; flashed from BOOTSEL) | uncommitted working tree on `2eb0a68` | Kiro (AI assistant), board connected by Jason | partial: U01–U22 as run 1; U11 NOT-RUN (test ran before the link was up) | `logs/2026-11-01_target_run2_partial.txt` (start of the run missing: two capture scripts overwrote each other), `logs/2026-11-01_broker_state_frames.txt`, `logs/2026-11-01_broker_cmds_sent.txt` | **First time the board reached SHARE_TELEMETRY**: joined the hotspot (172.20.10.3), MQTT connected 4.8 s after boot. Broker received 9-byte frames at 10 Hz (hand-decoded: time field +10 per frame). Over a real broker the board returned: pause ACCEPT; same pause DUPLICATE; 2-byte payload BAD_LENGTH; bad CRC BAD_CRC; type 0 UNKNOWN; version 2 BAD_VERSION; reserved bit BAD_RESERVED; old resume STALE; resume 12 ACCEPT; abort 13 ACCEPT and "REMOTE ABORT → flg_safety + remote_cmd_q"; 12-byte payload BAD_LENGTH. The duplicate abort was sent during a link drop and was lost (QoS 0): **not observed**. **4 unplanned link drops in 200 s** (62.9, 156.9, 178.8, 199.1 s), each `SOCK_TIMEOUT` 1500 ms after the last ack, each recovered in 0.6–1.1 s with no reset; cause not established (hotspot, delayed ACKs or firmware). 109 of ~2190 publishes (5 %) were refused as BUSY. Motors were not driven and Safety/VC do not exist, so "no unsafe movement" remains unproven. |
| 2026-11-01 | S1-U01…U24, run 3, first **cold power-up** (replug without BOOTSEL) | uncommitted working tree | Kiro (AI assistant), board connected by Jason | 21 PASS / 1 FAIL (U23); U11 NOT-RUN | `logs/2026-11-01_target_run3_coldboot_slow_timer.txt` | **Exposed a kernel defect.** `tk_dly_tsk(10)` lasted 468 µs on the 1 MHz TIMER; 2 s of motion ticks measured 47 ms (4267 ticks/s). The TIMER ran about 43× slow. Hypothesis, register **not read** on the board: `startup_clock()` never starts the watchdog tick generator (512 vs 12 cycles = 42.7×). Seen effects: join started at 6.8 s instead of 1.3 s, MQTT connect timeouts and retries, jitter probe period 202–267 µs (U23 FAIL). These failures follow from the slow timer; they are not separate firmware faults. Fix: `cpu_clock.c` writes `WDOG_TICK = ENABLE | 12`, `sysdef.h` defines it; test S1-U25 added. Fixed image built (sha256 `1ff71cad…`, 0 warnings) and loaded onto the board at 13:48:06. **Not yet verified on a cold boot.** |
| 2026-11-01 | run 4, cold power-up of the fixed image (sha256 `1ff71cad…`) | uncommitted working tree | Kiro (AI assistant), board replugged by Jason | timer fix **indirectly verified**; S1-U25 line not captured | `logs/2026-11-01_target_run4_coldboot_fixed.txt` | The capture connected ~3.5 min after boot; the 4 KB USB ring had overflowed, so the S1-U23…U25 lines were lost. Indirect evidence the TIMER now runs at 1 MHz after a cold boot: Wi-Fi join at t=1340 ms (run 3, slow timer: 6790 ms), `gap_max` = 260 664 µs on the TIMER (run 3: 6 549 µs; BOOTSEL runs: 259 458 µs), 50 polls/s on the kernel clock. The hotspot was off, so the board looped CONNECTING_WIFI → RECONNECTING ("SSID not found") with bounded back-off and no reset; backlog stayed at 16. S1-U25 itself still needs one captured run. |
| 2026-11-01 | S1-H01…H60 host revalidation | uncommitted working tree | Copilot SDK assistant | PASS 49/0; analyzer clean | `logs/2026-11-01_host_revalidation.txt` | WSL Ubuntu 24.04; ASan/UBSan and strict warnings. Logic-only evidence, not target evidence. |
| 2026-11-01 | six-profile firmware build matrix | uncommitted working tree | Copilot SDK assistant | PASS 6/6, exit 0, no build errors | `logs/2026-11-01_build_matrix_revalidation.txt` | Profiles: default, TEST=1, lwIP+MQTT, lwIP+MQTT+TEST, SMP=0 lwIP+MQTT, and CONSOLE=uart WIFI=none. Compile/link only; nothing flashed or run. |
| 2026-11-01 | `wifi_telemetry_tool.py selftest` | uncommitted working tree | Copilot SDK assistant | PASS | — | Host protocol-tool check; no target evidence. |
| 2026-11-01 | S1-U01…U25, run 6, normal replug motor image | uncommitted working tree | Copilot SDK assistant, board replugged by Jason | 24 PASS / 0 FAIL | `logs/2026-11-01_target_run6_normal_replug_motor.txt` | Captured target output: S1-U11 PASS (queue delay), S1-U23 PASS (jitter/no missed cycles), S1-U24 PASS, and S1-U25 PASS with `WATCHDOG TICK=2e0c` and 108306/109948/109980 us delays. One `SOCK_TIMEOUT` drop recovered in 620 ms during the first 10 s; later capture shows additional automatic recoveries, so controlled 10-trial link-loss testing remains open. |
| — | S1-M01…M05, M06 | — | — | NOT RUN | — | needs hardware, AP, broker, logic analyser |
