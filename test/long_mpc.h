#ifndef LONG_MPC_H_
#define LONG_MPC_H_
#include "handler.h"

bool LOG_MPC = std::getenv('LOG_MPC') ? true : false;
const float MPC_COST_LONG_TTC = 5.0;
const float MPC_COST_LONG_DISTANCE = 0.1;
const float MPC_COST_LONG_ACCELERATION = 10.0;
const float MPC_COST_LONG_JERK = 20.0;
const float _LEAD_ACCEL_TAU = 1.5;

typedef struct {
  double x_ego, v_ego, a_ego, x_l, v_l, a_l;
} state_t;


typedef struct {
  double x_ego[21];
  double v_ego[21];
  double a_ego[21];
  double j_ego[20];
  double x_l[21];
  double v_l[21];
  double a_l[21];
  double t[21];
  double cost;
} log_t;


class LongitudinalMpc
{
public:
    LongitudinalMpc();
    virtual ~LongitudinalMpc();
    void send_mpc_solution(PubSocket *livelongitudinalmpc_sock, int qp_iterations, int calculation_time);
    void setup_mpc();
    void set_cur_state(v, a);
    void update(Handler handler, PubSocket *livelongitudinalmpc_sock, LeadData lead);

private:
    int mpc_id;
    float v_mpc;
    float v_mpc_future;
    float a_mpc;
    float v_cruise;
    bool prev_lead_status;
    float prev_lead_x;
    bool new_lead;

    float last_cloudlog_t;

    float a_lead_tau;

    log_t mpc_solution;
    state_t cur_state;
};

#endif // LONG_MPC_H_