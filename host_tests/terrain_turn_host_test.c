#include <stdio.h>
#include <stdint.h>
#include "terrain_turn_logic.h"
#include "imu_math.h"
#define T(x) do { if (!(x)) { printf("FAIL line=%d: %s\n", __LINE__, #x); failures++; } } while(0)
int main(void)
{
 int failures=0, i; float diff=0, peak=0; int started=0, valid_done=0;
 b4_turn_checker_t turn={0}; imu_hump_t hump;
 T(b4_turn_begin(&turn,350,45,1000,3000));
 T(b4_turn_check(&turn,44,34,1,1800,&diff)==B4_TURN_COMPLETE_AGREED);
 T(b4_turn_check(&turn,45,75,1,1800,&diff)==B4_TURN_TRACKING);
 T(b4_turn_check(&turn,45,75,1,4100,&diff)==B4_TURN_COMPLETE_ENCODER_FALLBACK);
 T(b4_turn_check(&turn,20,75,1,4100,&diff)==B4_TURN_TIMEOUT);
 T(b4_turn_begin(&turn,20,-90,0,3000));
 T(b4_turn_check(&turn,-90,290,1,1000,&diff)==B4_TURN_COMPLETE_AGREED);
 T(b4_terrain_request(1,1,1)==B4_TERRAIN_SLOW_REQUEST);
 T(b4_terrain_request(1,0,1)==B4_TERRAIN_HALT);
 imu_hump_reset(&hump);
 for(i=0;i<15;i++){imu_hump_event_t e=imu_hump_update(&hump,10.0f,3.0f,&peak); if(e==IMU_HUMP_STARTED)started++;}
 for(i=0;i<15;i++)(void)imu_hump_update(&hump,-10.0f,3.0f,&peak);
 for(i=0;i<4;i++){imu_hump_event_t e=imu_hump_update(&hump,0.0f,0.0f,&peak); if(e==IMU_HUMP_DONE_VALID)valid_done++;}
 T(started==1);T(valid_done==1);T(peak >= 2.0f);
 printf("[HOST] HUMP-UP-DOWN + TURN FUSION: %s (%d failures, peak~%.2fmm)\n",failures?"FAIL":"PASS",failures,peak);
 return failures!=0;
}
