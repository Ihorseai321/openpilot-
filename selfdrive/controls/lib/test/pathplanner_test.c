#include <stdio.h>
#include "../lateral_mpc/lateral_mpc.h"
#define DEG_TO_RAD 3.1415926535898 / 180


void hello()
{
    printf("hello world!!!\n");
}

void calc_states_after_delay(state_t *states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay)
{
    states->x = v_ego * delay;
    states->psi = v_ego * curvature_factor * steer_angle * DEG_TO_RAD / steer_ratio * delay;
}
