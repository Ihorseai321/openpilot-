#ifndef LONG_MPC_H_
#define LONG_MPC_H_
#include "handler.h"
#include "utils.h"
#include "messaging.hpp"
#include "mpc_structs.h"


const float MPC_COST_LONG_TTC = 5.0;
const float MPC_COST_LONG_DISTANCE = 0.1;
const float MPC_COST_LONG_ACCELERATION = 10.0;
const float MPC_COST_LONG_JERK = 20.0;
const float _LEAD_ACCEL_TAU = 1.5;


class LongitudinalMpc
{
public:
    LongitudinalMpc(int mpc_id);
    virtual ~LongitudinalMpc();
    void send_mpc_solution(PubSocket *livelongitudinalmpc_sock, int qp_iterations, int calculation_time);
    void setup_mpc();
    void set_cur_state(float v, float a);
    void update(Handler handler, PubSocket *livelongitudinalmpc_sock, LeadData lead);

    bool prev_lead_status;
    float v_mpc;
    float a_mpc;
    int mpc_id;
    float v_mpc_future;
    float v_cruise;
    float prev_lead_x;
    bool new_lead;

    float last_cloudlog_t;

    float a_lead_tau;

    long_log_t mpc_solution;
    long_state_t cur_state;
};

#endif // LONG_MPC_H_