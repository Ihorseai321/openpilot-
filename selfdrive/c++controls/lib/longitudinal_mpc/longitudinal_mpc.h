#ifndef LONGITUDINAL_MPC_H_
#define LONGITUDINAL_MPC_H_
#include "../mpc_structs.h"

void long_init(float ttcCost, float distanceCost, float accelerationCost, float jerkCost);
void long_init_with_simulation(float v_ego, float x_l, float v_l, float a_l, float l);
int long_run_mpc(long_state_t * x0, long_log_t * solution, float l, float a_l_0);

#endif // LONGITUDINAL_MPC_H_