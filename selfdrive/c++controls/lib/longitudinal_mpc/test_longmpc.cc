#include "longitudinal_mpc.h"


int main(int argc, char *argv[])
{
    float MPC_COST_LONG_TTC = 5.0;
    float MPC_COST_LONG_DISTANCE = 0.1;
    float MPC_COST_LONG_ACCELERATION = 10.0;
    float MPC_COST_LONG_JERK = 20.0;
    long_init(MPC_COST_LONG_TTC, MPC_COST_LONG_DISTANCE, MPC_COST_LONG_ACCELERATION, MPC_COST_LONG_JERK);
    return 0;
}