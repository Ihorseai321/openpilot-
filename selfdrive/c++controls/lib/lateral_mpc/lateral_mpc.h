#ifndef LATERAL_MPC_H_
#define LATERAL_MPC_H_
#include "../mpc_structs.h"

void lateral_init(float pathCost, float laneCost, float headingCost, float steerRateCost);
void lateral_init_weights(float pathCost, float laneCost, float headingCost, float steerRateCost);
int lateral_run_mpc(lateral_state_t *x0, lateral_log_t *solution, float l_poly[4], float r_poly[4], float d_poly[4], 
            float l_prob, float r_prob, float curvature_factor, float v_ref, float lane_width);
#endif //LATERAL_MPC_H_