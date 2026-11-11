# Subsystem 4 — test log

**Subsystem:** Monitor (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | interim safety + vehicle tasks start, no hold → DRIVING, line gate open | IF-B4-01 | auto | `subsystem4_test.c` (test starts the tasks itself) | `[TEST] S4-U03 PASS` |
| S4-U04 | remote PAUSE → PAUSED, gate closed, line post refused `E_OBJ` | IF-B1-02, IF-B4-01 | auto | test posts PAUSE to `remote_cmd_q` | `[TEST] S4-U04 PASS` |
| S4-U05 | remote RESUME → DRIVING, gate open | IF-B1-02 | auto | test posts RESUME | `[TEST] S4-U05 PASS` |
| S4-U06 | obstacle STOP and TURN_LEFT hold the car, CONTINUE releases | IF-06 | auto | test sends 3 events to `mbf_obstacle` | `[TEST] S4-U06 PASS` |
| S4-U07 | `S5_FLG_OBSTACLE_STOP` holds while set, releases when cleared (never latched) | IF-07 | auto | test sets/clears the bit | `[TEST] S4-U07 PASS` |
| S4-U08 | `SAFETY_REQ_REMOTE_ABORT` → Safety clears it and latches ESTOP; VC → ERROR, gate closed | IF-B1-03/04 | auto | test sets the request bit | `[TEST] S4-U08 PASS` |
| S4-U09 | remote ABORT is sticky: latch reset + RESUME do not restart | IF-B1-02 | auto | test posts ABORT, clears latch, posts RESUME | `[TEST] S4-U09 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
