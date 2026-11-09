#include <stdio.h>
#include "safety_phase4c_test.h"
int main(void) {
    int n = safety_phase4c_test_run();
    printf("P4C offline policy: %s (%d failed)\n", n ? "FAIL" : "PASS", n);
    return n ? 1 : 0;
}
