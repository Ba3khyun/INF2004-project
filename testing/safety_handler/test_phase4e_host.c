#include <stdio.h>
#include "safety_phase4e_test.h"
int main(void) {
    int failures = safety_phase4e_test_run();
    printf("P4E-HANDOFF %s (%d failed; MODEL ONLY)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
