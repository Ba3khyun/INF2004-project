# Pico W Robot — micro T-Kernel 3.0 (SMP)

A five-member embedded robotics project running **micro T-Kernel 3.0 on the Raspberry Pi Pico W (RP2040)**. The firmware combines Wi-Fi telemetry, motor and encoder control, IR line following and barcode recognition, an LSM303D IMU, and a Vehicle Controller/Safety Handler. Code is organized into student-owned subsystems and shared RTOS interfaces.

**Documentation scope:** This README describes the **whole repository**, including capabilities already present in branch v1 and changes verified in branch v2 and branch v4. A feature being documented here does **not** imply it was implemented in the most recent commit or by Buddy 4. For ownership, pin claims, and interface agreements, maintain [`TEAM.md`](TEAM.md) separately.

> **Current integration/safety status:** The Vehicle Controller's motor output is **passive**; it does not forward driving commands to Buddy 2. `SH_ENABLE_SHARED_ESTOP_ASSERT` remains **0**. Safety tests validate software policies/read-only behavior, **not a physical emergency stop**. Keep motor power disconnected during software-only validation.

## 1. Project status at a glance

| Owner | Module | Present in branch v4 | Status / boundary |
| --- | --- | --- | --- |
| **Buddy 1** | Wi-Fi commands and telemetry — `app_program/subsystem1/` | Wi-Fi/MQTT state machine, telemetry queue/backlog, command parser, remote-abort signaling, optional LED example | Firmware logic exists; live network connection depends on build options and AP/broker setup |
| **Buddy 2** | Motion/encoders — `app_program/subsystem2/` | Dual-motor PWM, encoder interrupts, wheel-distance/speed calculations, PID helpers, 100 Hz motion task, motor/ESTOP checks | Motor-disabled tests run; physical driving not validated by Buddy 4's suite |
| **Buddy 3** | Line following/barcode — `app_program/subsystem3/` | Two analog IR line sensors, line recovery/steering, GPIO-interrupt barcode reader, Vehicle Controller intent handoff | Steering path redirected to **passive** Vehicle Controller; actual motor movement through that path is disabled |
| **Buddy 4** | IMU — `app_program/subsystem4/` | LSM303D driver, calibration, heading/pitch/roll, shock/hump event logic, packed snapshot, 100 Hz periodic sampling | Sensor identification/monitoring/timing tests passed on Pico W |
| **Buddy 4** | Vehicle Controller — `app_program/vehicle_controller/` | State/priority logic and RTOS adapter; atomic line-intent handoff, IMU/safety observations | **Read-only/passive**, no motor commands |
| **Buddy 4** | Safety Handler — `app_program/safety_handler/` | RTOS observer, sticky software latch, ESTOP policy/bridge models and combined tests | Live shared-ESTOP assertion and reset coordination **not enabled** |
| **Buddy 5** | Planned obstacle scanning — `app_program/subsystem5/` | Initialization/start stubs and a placeholder test only | **Not yet implemented in uploaded v4 source**; obstacle request exists in shared safety contract/model |

### Runtime interaction (current versus planned)

```text
IR line sensors / barcode (Buddy 3)
           |
           | vehicle_controller_line_motion_post(command, line_visible)
           v
Vehicle Controller (Buddy 4, passive) <--- IMU snapshot (Buddy 4)
           |                         <--- flg_safety (shared)
           | proposed motion intent only
           X  [NOT connected to Buddy 2 command queue]

Buddy 2 motion task <--- motion_cmd_q (used by tests / existing callers)
       |                   |
       |                   +--- checks SAFETY_ESTOP_LATCHED
       +--- motor PWM + encoders + speed telemetry

Buddy 1 Wi-Fi ----> remote_cmd_q (not yet consumed by VC)
             \----> SAFETY_REQ_REMOTE_ABORT (shared flag)
Buddy 5 [planned] -> SAFETY_REQ_OBSTACLE_STOP
IMU shock events --> SAFETY_REQ_COLLISION
                          |
                          v
Safety Handler (software observer/policy; live ESTOP assertion OFF)
```

The blocked connection is intentional: **line-following decisions are not currently sent through the Vehicle Controller to the motors**.

## 2. Hardware, pins and responsibilities

The Pico W runs on a Robo Pico / Cytron Maker Pi Pico-style base. Pin allocation below is derived from the **source definitions**, not a substitute for checking the actual wiring and resolving ownership in `TEAM.md`.

| Pins / interface | Current source use | Owner |
| --- | --- | --- |
| **GP4 SDA, GP5 SCL** / I²C0 | LSM303D accelerometer + magnetometer, Grove Port 3 | Buddy 4 |
| **GP8–GP9** / PWM slice 4 | Left motor outputs | Buddy 2 |
| **GP10–GP11** / PWM slice 5 | Right motor outputs | Buddy 2 |
| **GP6 (right A), GP7 (left A)** | Wheel encoder pulse inputs, rising-edge interrupts | Buddy 2 |
| GP16 (right B), GP17 (left B) | Listed in the motion driver pin documentation; current count implementation uses A-channel rising edges | Buddy 2 |
| **GP26 (ADC0), GP27 (ADC1)** | Right and center IR line sensors | Buddy 3 |
| **GP28** | Barcode digital input, shared GPIO interrupt dispatch | Buddy 3 / IRQ cooperation with Buddy 2 |
| GP19 | Legacy external LED heartbeat | Buddy 1/example |
| GP23–GP25, GP29 | Reserved for Pico W wireless hardware | Board |

**Code/documentation mismatch to fix:** `subsystem3.h` defines `BARCODE_PIN 28`, but an adjacent comment refers to GP17 and some serial messages say GP2. The **compiled pin constant is GP28**; verify the actual board wiring before testing. The encoder's B-channel pins are documented but should not be described as fully decoded quadrature in this version.

## 3. Subsystem implementations

### Buddy 1 — Wi-Fi communication, remote commands and telemetry

**Files:** `subsystem1/wifi_telemetry.c/.h`, `wifi_sm.c/.h`, `wifi_cmd.c/.h`, `subsystem1.c/.h`.

- A bounded **16-message telemetry queue** accepts non-blocking four-byte status updates via `telemetry_post()`; when full, it drops older entries rather than blocking real-time tasks.
- Supports status fields for Vehicle Controller state, speed, IMU hump and obstacle range (the presence of a field does not prove its producer is active).
- Builds telemetry frames and maintains a backlog, connection/reconnect state machine and Wi-Fi/MQTT command parsing with sequence/CRC checks.
- Handles **abort, pause and resume** commands. Abort can raise `SAFETY_REQ_REMOTE_ABORT`; `remote_cmd_q` is available for the Vehicle Controller but is not consumed by its current RTOS adapter.
- The Wi-Fi service task is priority **40**; Wi-Fi/lwIP/MQTT operations must remain inside the designated network-owning task.
- The original external LED task remains as a compatibility/example component.

**Network testing caveat:** The uploaded `pico-w-test` CMake preset explicitly uses `NET=none` and `MQTT=0`. This does **not** enable live MQTT networking; a disconnected state and `S1-U24` failure in that build should not by themselves be treated as proof of a broken radio implementation. Live connection testing needs an appropriate network-enabled build and configured access point/broker.

### Buddy 2 — Motion control, wheel encoders and PID helpers

**Files:** `subsystem2/motion_control.c/.h`, `task_motion.c`, `subsystem2.c/.h`.

- Controls **two motors** using RP2040 PWM slices 4 and 5 (GP8–GP11), configured for **20 kHz** PWM; includes forward/reverse drive, pivots and braking.
- Initializes **right/left wheel encoder** A-channel inputs on GP6/GP7. A shared GPIO interrupt handler counts rising edges; the module also dispatches an auxiliary GPIO interrupt callback used by Buddy 3's barcode reader.
- Provides encoder pulse-to-revolution and wheel-distance conversion utilities and accumulated travel distance (`motion_get_distance_mm()`).
- Provides reusable **PID controller functions** (`pid_init`, `pid_update`, `pid_set_limits`). These helpers exist in the driver; do **not** assume a fully tuned closed-loop speed controller is active in the main motion task.
- A **priority-10, core-1 motion task** is awakened by a **10 ms cyclic handler** (~100 Hz) and receives one-byte commands via `motion_cmd_q`.
- Supports discrete **F/L/R**, continuous forward/pivot/steering intents, brake/halt and local stop (`X`) / clear (`C`). The task checks `SAFETY_ESTOP_LATCHED` before or during movement and can set `FLT_MOTOR` for timeouts.
- Reports speed telemetry approximately every **100 ms** (`TQ_FIELD_SPEED`).
- **Wheel diameter change verified between v1 and v2/v4:** `WHEEL_DIAMETER_CM` changed **6.5f → 6.2f** in `task_motion.c`. Confirm the physical wheel measurement with Buddy 2.

**Important provenance:** The motor PWM, encoders, interrupt handling, distance helpers, PID utilities and motion task **already exist in the uploaded branch v1**. They are essential project features but **were not newly introduced by the v1→v4 Buddy 4 integration diff**.

### Buddy 3 — IR line following, barcode and steering intent

**Files:** `subsystem3/task_consumer.c`, `subsystem3.c/.h`.

- Reads the **right (GP26/ADC0)** and **center (GP27/ADC1)** IR line sensors, applies a threshold and hysteresis, and tracks line-visible state.
- Implements line-following, lost-line recovery and barcode-reading states. The GPIO interrupt on **GP28** captures barcode signal edges and pulse widths; the source recognizes **A–D** patterns.
- Includes steering/line-recovery decisions such as forward, gentle steering and a recovery pivot, as well as a periodic line-intent refresh.
- The task continues to service **legacy sample/statistics interfaces** (`mbx_samples`, `app_stats`) inherited from the producer-consumer teaching example. These are separate from the actual line-control integration.

**Changes verified in v1→v2/v4:** `task_consumer.c` includes the Vehicle Controller header, replaces **six** `motion_cmd_post(...)` steering call sites with `vehicle_controller_line_motion_post(command, right_line || center_line)`, and adds **one additional heartbeat/refresh call** when recovering a lost line. This is **six replacements + one new call**, not seven replacements.

**Behavioral consequence:** Those paths no longer call Buddy 2 directly. As the Vehicle Controller does **not forward output commands**, the redirected steering paths cannot currently drive the robot. Buddy 3 should review and approve these cross-owner edits before merging into newer work.

### Buddy 4 — LSM303D IMU

**Files:** `subsystem4/imu_lsm303.c/.h`, `imu_math.c/.h`, `imu_task.c`, `subsystem4.c/.h` and `subsystem4_test.c`.

- Uses I²C0 (GP4/GP5) and identifies the **LSM303D** by its `WHO_AM_I` register `0x0F` returning **`0x49`**; the driver supports possible addresses **0x1D/0x1E**.
- Configures and reads three-axis accelerometer and magnetometer measurements, with ±4 g / ±2 gauss range settings and scale conversion.
- Samples on a **10 ms cyclic tick** (~100 Hz), performs stationary calibration, and reports heading, pitch, roll, acceleration magnitude, magnetic magnitude and read errors.
- Includes collision/shock and hump-related processing, an **8-message IMU event buffer**, a packed single-word sensor snapshot and sensor/fault/timing statistics.
- Provides read-only snapshot/status access so other tasks do **not** directly access I²C0.
- Keeps an older example `monitor_task` and summary-message interfaces for compatibility with the original project template. Its verbose logging is gated in v4.

**Verified v1→v4 change:** `imu_task.c` gained startup return-code logging for `imu_drv_init()`, and `task_monitor.c` gained `S4_MONITOR_DEBUG_PRINT` gates. The underlying LSM303D driver and IMU algorithms are **already present in v1**, so document them as project functionality rather than claiming they were all added in v4.

### Buddy 4 — Vehicle Controller

**Files:** `vehicle_controller/vehicle_controller_core.c/.h`, `vehicle_controller.c/.h` (**added after v1**).

- Implements state-based, safety-prioritized decisions using IMU health/snapshot, shared safety flags, line visibility and recent line-following intents.
- Models initialization, driving, line search/loss, staleness timeout, pause, hump-related handling, mission completion and error/halt priority.
- Uses an atomic line-intent handoff from Buddy 3, and prints passive diagnostic updates such as `[VC] state=... line=... intent=... output=...`.
- **Deliberately does not call `motion_cmd_post()` to drive the motors.** It does not yet consume `remote_cmd_q` or complete the production IMU-event-to-action chain.

### Buddy 4 — Safety Handler

**Files:** `safety_handler/safety_core.c/.h`, `safety_handler.c/.h`, `safety_policy.c/.h`, `safety_estop_bridge.c/.h`, `safety_handler/tests/` (**added after v1**).

- A **priority-5 observer task**, currently polling every **50 ms**, inspects collision, obstacle-stop and remote-abort requests via the shared `flg_safety` event flag.
- Tracks a **sticky software latch**, reasons and conservative observer-failure state.
- Models ESTOP assertions, ownership, authorization and rejection of unsafe resets; provides an assert-only bridge with regression tests.
- **`SH_ENABLE_SHARED_ESTOP_ASSERT=0` in v4**: the observer does **not** currently set/clear the live `SAFETY_ESTOP_LATCHED` bit in production. There is no validated physical stop/reset workflow.
- Test implementations are organized under `safety_handler/tests/`, invoked once from `safety_test_suite_run()` (the Phase 4A–4F software/read-only regression set).

### Buddy 5 — Obstacle scanning (planned)

**Files in uploaded v4:** `subsystem5/subsystem5.c/.h`, `subsystem5_test.c`.

The checked v4 source contains **only an empty initialization/start skeleton and a placeholder `S5-U01` test**. There is **no live obstacle-scanning task or sensor driver in this snapshot**. `SAFETY_REQ_OBSTACLE_STOP` is defined in shared data and used by Buddy 4's policy/model tests; actual publication and acknowledgement by Buddy 5 remain to be integrated.

## 4. Shared RTOS interfaces

| Interface | Writer / provider | Reader / consumer | Actual integration status |
| --- | --- | --- | --- |
| `motion_cmd_q`, `motion_cmd_post()` | Motion command producers / tests | Buddy 2 motion task | Existing queue and task; Vehicle Controller **not yet forwarding** |
| `vehicle_controller_line_motion_post()` | Buddy 3 line task | Buddy 4 Vehicle Controller | **Connected; passive** |
| `s4_imu_get_snapshot()` | Buddy 4 IMU | Vehicle Controller / tests | **Connected, read-only** |
| `flg_safety`, `SAFETY_REQ_*` | IMU / Wi-Fi / planned obstacle sender | Safety Handler / VC / Buddy 2 | Shared definition exists; live delivery and acknowledgement need end-to-end review |
| `SAFETY_ESTOP_LATCHED` | Intended Safety Handler owner | Buddy 2 / VC / Wi-Fi | Buddy 2 test proves refusal when set; **live handler assertion disabled** |
| `telemetry_q`, `telemetry_post()` | Motion/IMU/VC/obstacle producers | Buddy 1 Wi-Fi task | Queue works in test; network publishing requires configured build |
| `remote_cmd_q` | Buddy 1 | Planned Vehicle Controller consumer | Defined and tested at queue/command level; **not consumed by VC yet** |
| `mbf_imu_evt`, `s4_imu_snapshot` | Buddy 4 IMU | Future consumers / tests | IMU event/snapshot interfaces provided; full action integration pending |
| `mbx_samples`, `mbf_summary`, `app_stats` | Template producer/consumer/monitor | Template tasks/tests | **Legacy teaching-example path**, not equivalent to real robot integration |

For official ownership and bit definitions see `app_program/app_data.h` and [`TEAM.md`](TEAM.md). The existing `TEAM.md` still has some outdated example rows that the team should update.

## 5. What changed between project snapshots?

This is a **source comparison**, not Git authorship attribution. It describes what changed in the uploaded archives, not who personally made each change.

| Area | v1 → v2 | v2 → v4 |
| --- | --- | --- |
| Buddy 1 (`subsystem1/`) | No subsystem file changes | No subsystem file changes |
| Buddy 2 (`subsystem2/`) | `task_motion.c`: wheel diameter **6.5 → 6.2 cm** | No subsystem file changes |
| Buddy 3 (`subsystem3/`) | `task_consumer.c`: six motion calls redirected to VC + new lost-line refresh | No subsystem file changes |
| Buddy 4 IMU (`subsystem4/`) | IMU startup debug print and monitor logging switch | No subsystem file changes |
| Buddy 4 Vehicle Controller | Added `vehicle_controller/` decision core and passive adapter | Kept |
| Buddy 4 Safety Handler | Added core, observer and initial validation | Added policy/bridge and renamed, consolidated 4A–4F tests |
| `app_main.c` | Added fail-fast object initialization and Safety/VC init/start | Switched to one Buddy 4 test-suite call |
| `CMakeLists.txt` | No change | Added nested `safety_handler/tests/*.c` sources |
| Shared kernel/config | Task-priority ceiling **32 → 48**, bitmap helpers, watchdog timer tick setup, I²C debug-print changes | Same underlying changes retained |
| Buddy 5 (`subsystem5/`) | No subsystem file changes | No subsystem file changes |
| README files | Phase-level notes added | Consolidated README; phase-level notes removed |

**Additional shared-code details from v1→v4:**

- `config/config.h`: `CNF_MAX_TSKPRI` raised to **48** to accommodate Wi-Fi priority 40.
- `config/config_func.h`: enabled bitmap helper functions used by expanded priority handling.
- `kernel/sysdepend/pico_rp2040/hw_setting.c`: initializes watchdog tick generator for the RP2040 timer's **1 MHz** microsecond reference.
- `device/i2c/sysdepend/rp2040/i2c_rp2040.c`: added switch to suppress verbose I²C debug output (existing low-level driver otherwise retained).
- `app_program/app_main.c`: aborts application task startup if kernel-object initialization fails, then initializes/starts all five subsystems plus the Safety Handler and Vehicle Controller.

**Review requirement:** Changes under `subsystem2/`, `subsystem3/`, `config/`, `kernel/`, `device/` and shared app files should be reviewed by their respective owners before merging. A ZIP comparison cannot establish which person authored a change.

## 6. Build, flash and monitor

### Requirements

- Raspberry Pi Pico W / RP2040 board, USB cable and the repository source.
- Pico SDK (including USB/Wi-Fi submodules where required), Arm GNU `arm-none-eabi-gcc`, CMake **3.25+**, Ninja and a serial monitor.
- Windows users may use the Pico VS Code extension; macOS/Linux/WSL may use their installed Pico SDK and toolchain. Set `PICO_SDK_PATH` and the toolchain path appropriately.

**Important:** The uploaded `CMakePresets.json` contains **absolute Windows paths from one team member's PC**. Edit the SDK/toolchain/Ninja paths or override them for your own machine; do not assume the preset works unchanged everywhere.

### CMake (recommended for this branch)

```sh
# Build firmware that executes subsystem + Buddy 4 regression tests
cmake --preset pico-w-test
cmake --build --preset pico-w-test

# Regular firmware build (no test runner)
cmake --preset pico-w
cmake --build --preset pico-w
```

- The self-test build uses the `build-test/` directory; the ordinary firmware uses `build/`.
- Select the generated `.uf2`, reboot into BOOTSEL mode and copy it to the Pico W's `RPI-RP2` USB drive.
- Open USB serial at **115200 baud** (for example, VS Code Serial Monitor or PuTTY on Windows).
- Keep motor power disconnected for the current integration checks.

For the original Make-based build, the upstream repository supports `make TEST=1 -j8` from `build_make/`, but its source list may **not** automatically include the new nested `safety_handler/tests/` directory; confirm or update that build path before using it.

### Wi-Fi / MQTT option

The stock `pico-w-test` preset has `NET=none` and `MQTT=0`, so it is appropriate for **offline firmware/unit testing**, not evidence of a live broker connection. A network-enabled build requires compatible `NET=lwip` and `MQTT=1` configuration **plus** approved AP/broker details. Do not publish credentials in the README or Git repository.

## 7. Testing and observed results

### Buddy 4 one-shot regression (Pico W)

User-confirmed after the test-file cleanup: the rebuilt **Buddy 4 combined suite passed without failures**. A captured prior run recorded:

```text
[B4-ALL] SH-CORE               PASS (failed=0)
[B4-ALL] P4B-READONLY          PASS (failed=0)
[B4-ALL] P4C-POLICY            PASS (failed=0)
[B4-ALL] P4D-BRIDGE            PASS (failed=0)
[B4-ALL] P4E-HANDOFF           PASS (failed=0)
[B4-ALL] VC + SAFETY SCENARIOS PASS (failed=0)
[B4-ALL] LIVE READ-ONLY HEALTH PASS (failed=0)
[B4-ALL] FINAL: PASS (0 / 7 groups failed; MOTOR DISABLED)
```

The suite includes software safety policy/bridge and VC scenario checks, plus **read-only** observations of live IMU sampling, safety flags and passive VC state. It does **not** prove active ESTOP or powered robot motion.

### IMU timing (captured self-test run)

| Measurement | Observed | Requirement/check |
| --- | --- | --- |
| Cycles in 3 seconds | **301** | Around 300; 100 Hz target |
| Worst execution time | **2,063 µs** | Under 5,000 µs |
| Worst cycle interval | **10,447 µs** | Under 20,000 µs |
| Cycles in 500 ms health check | **51** | Sampling remains active |
| Sensor state | `MONITORING`, snapshot valid | Read-only IMU health passed |

### Other tests / limitations

- Buddy 2's **ESTOP F/L/R rejection**, motor-PWM-zero checks and 100 Hz tick checks have passed in uploaded user logs; powered motor behavior was **not tested** in the Buddy 4 batch (`S2_TEST_MOTORS=0`).
- `S2-U04` (motion-task sleep state) has passed on some runs and failed on others; investigate the sampling/state-check timing independently.
- `S1-U24` frequently reports `DISCONNECTED`; the offline test preset's `NET=none, MQTT=0` should be considered before attributing it to the Wi-Fi implementation.
- `INT-01` (example S2 → S3 sample flow) and `INT-02` (example S3 → S4 summary flow) have failed in the supplied logs. **Both tests and the underlying example interfaces are present in uploaded v4**. If the team removes them in a later working tree, remove or replace the corresponding dependent example code deliberately rather than assuming all of it is unused.
- Buddy 5's passing `S5-U01` is only a **placeholder task-context test**; it does not validate obstacle sensing.
- No evidence in the uploaded source/logs establishes successful end-to-end Wi-Fi/MQTT connection, physical line-course traversal, obstacle avoidance, hump traversal or physical ESTOP actuation.

## 8. Integration work still required

1. Agree a **single motor-command owner**, then implement and verify Vehicle Controller → Buddy 2 arbitration. Check any remaining direct-motion paths with Buddy 3.
2. Provide persistent, acknowledged collision/obstacle/abort requests so a brief event cannot disappear between Safety Handler polls (currently **50 ms**).
3. Implement and independently verify active ESTOP assertion, latch ownership, explicit authorized reset and Buddy 2 stopping behavior; keep `SH_ENABLE_SHARED_ESTOP_ASSERT=0` until reviewed.
4. Integrate Buddy 5's actual obstacle sensor/task and event producer; finish remote command consumption and IMU event handling.
5. Verify the compiled pin mapping and shared GPIO IRQ arrangement; validate line/barcode/encoder behavior together using suitable safe bench tests.
6. Replace obsolete template sample/statistics tests with integration tests for the **real** subsystem interfaces, with team agreement.
7. After motor-disabled checks and a safety review, perform supervised acceptance tests of the assembled robot under an approved testing procedure.

## 9. Repository documentation and collaboration

- **`README.md` (this file):** implementation, build, verified changes across versions, test results and known limitations.
- **`TEAM_INTEGRATIOn.md`:** team-owned register of subsystems, pins, peripherals, priorities and shared interface IDs. **Keep it separately**; update outdated template rows together rather than replacing it with this README.
- **`testing/` and test logs:** evidence and regression history. The live Buddy 4 combined runner is `app_program/safety_handler/tests/safety_test_suite.c`.
- **Pull requests / Git diffs:** review cross-owner edits. In particular, Buddy 3's changed steering handoff is a functional behavioral change, while Buddy 2's wheel-diameter adjustment needs owner confirmation.

---
