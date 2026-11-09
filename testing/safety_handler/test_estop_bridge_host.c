#include <stdio.h>
#include "../app_program/safety_handler/safety_estop_bridge.h"
#define T(expr) do {if (!(expr)) {printf("FAIL line %d: %s\n",__LINE__,#expr);return 1;}}while(0)
int main(void)
{
    seb_state_t s;
    seb_init(&s);
    T(!seb_step(&s, 0, 1));
    T(seb_step(&s, SEB_COLLISION, 1));
    T(seb_step(&s, 0, 1));
    T(s.reasons == SEB_COLLISION);
    seb_init(&s);
    T(seb_step(&s, SEB_OBSTACLE, 1));
    seb_init(&s);
    T(seb_step(&s, SEB_REMOTE_ABORT, 1));
    seb_init(&s);
    T(seb_step(&s, 0, 0));
    T(s.read_failed && seb_step(&s, 0, 1));
    seb_init(&s);
    T(seb_step(&s, SEB_ESTOP, 1));
    T(seb_step(NULL, 0, 1));
    puts("P4D bridge host: 11 assertions PASS (no flag writes)");
    return 0;
}
