#include "safety_handler/safety_validation.h"
#include <stdio.h>
int main(void) {
    int failed = safety_validation_run();
    printf("[TEST] SH-CORE %s (%d failed)\n", failed ? "FAIL" : "PASS", failed);
    return failed ? 1 : 0;
}
