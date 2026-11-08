#include <stdio.h>
#include "safety_phase4f_checks.h"
int main(void) {
  int n = safety_phase4f_checks_run();
  printf("P4F pure-C host checks: %s (%d failures)\n", n ? "FAIL" : "PASS", n);
  return n ? 1 : 0;
}
