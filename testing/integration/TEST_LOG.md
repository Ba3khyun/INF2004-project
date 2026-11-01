# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-B1-01 | Wi-Fi + Vehicle Controller: pause/resume/abort requests arrive in `remote_cmd_q` and the VC changes mission state; VC never acts on RESUME while ESTOP is latched | IF-B1-02 | manual | publish commands with `tools/wifi_telemetry_tool.py` + `mosquitto_pub`; watch telemetry state field | correct state; no movement on rejected commands. **Blocked**: VC code missing |
| INT-B1-02 | Wi-Fi + Safety: remote abort → ESTOP latched → PWM 0 ≤ 50 ms; resume rejected until local reset at GP3 | IF-B1-03/04, NFR2.3 | manual | scope MQTT-RX GPIO toggle vs motor PWM pins | ≤ 50 ms, stays stopped. **Blocked**: Safety + Motion code missing |
| INT-B1-03 | Wi-Fi + telemetry producers: VC, Motion, IMU, Obstacle call `telemetry_post` at their real rates; frame fields match | IF-B1-01 | manual | `decode --stdin` on the broker | values match, residency ≤ 50 ms, no `E_QOVR`. **Blocked** |
| INT-B1-04 | Wi-Fi outage while motors and sensors are active: 30 s AP off during a run | IF-B1-01, NFR1.2 | manual | logic analyser on Motion/Line GPIO toggles | jitter ≤ 5 ms, no missed cycle, telemetry recovers. **Blocked** |
| INT-B1-05 | Remote abort while all tasks are active | IF-B1-02/03 | manual | as INT-B1-02 with line following + obstacle scanning running | stop ≤ 50 ms. **Blocked** |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
