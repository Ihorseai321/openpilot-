#include <cmath>
#include "long_mpc.h"
#include "utils.h"

LongitudinalMpc::LongitudinalMpc(int mpc_id)
{
    this.mpc_id = mpc_id;

    setup_mpc();
    v_mpc = 0.0;
    v_mpc_future = 0.0;
    a_mpc = 0.0;
    v_cruise = 0.0;
    prev_lead_status = false;
    prev_lead_x = 0.0;
    new_lead = false;

    last_cloudlog_t = 0.0;
}

LongitudinalMpc::~LongitudinalMpc()
{}

void LongitudinalMpc::setup_mpc()
{
    init(MPC_COST_LONG_TTC, MPC_COST_LONG_DISTANCE, MPC_COST_LONG_ACCELERATION, MPC_COST_LONG_JERK)

    cur_state.v_ego = 0;
    cur_state.a_ego = 0;
    a_lead_tau = _LEAD_ACCEL_TAU;
}

void LongitudinalMpc::set_cur_state(v, a)
{
    cur_state.v_ego = v;
    cur_state.a_ego = a;
}

void LongitudinalMpc::send_mpc_solution(PubSocket *livelongitudinalmpc_sock, int qp_iterations, int calculation_time)
{
    capnp::MallocMessageBuilder mpc_msg;
    cereal::Event::Builder event = mpc_msg.initRoot<cereal::Event>();
    event.setLogMonoTime(nanos_since_boot());
    auto long_mpc_send = event.initLiveLongitudinalMpc();

    kj::ArrayPtr<const float> x_ego_send(&mpc_solution.x_ego[0], 21);
    kj::ArrayPtr<const float> v_ego_send(&mpc_solution.v_ego[0], 21);
    kj::ArrayPtr<const float> a_ego_send(&mpc_solution.a_ego[0], 21);
    kj::ArrayPtr<const float> x_lead_send(&mpc_solution.x_l[0], 21);
    kj::ArrayPtr<const float> v_lead_send(&mpc_solution.v_l[0], 21);

    long_mpc_send.setXEgo(x_ego_send);
    long_mpc_send.setVEgo(v_ego_send);
    long_mpc_send.setAEgo(a_ego_send);
    long_mpc_send.setXLead(x_lead_send);
    long_mpc_send.setVLead(v_lead_send);

    long_mpc_send.setCost(mpc_solution.cost);
    long_mpc_send.setALeadTau(a_lead_tau);
    long_mpc_send.setMpcId(mpc_id);
    long_mpc_send.setQpIterations(qp_iterations);
    long_mpc_send.setCalculationTime(calculation_time);

    auto words = capnp::messageToFlatArray(mpc_msg);
    auto bytes = words.asBytes();
    livelongitudinalmpc_sock->send((char*)bytes.begin(), bytes.size());
}

void LongitudinalMpc::update(Handler handler, PubSocket *livelongitudinalmpc_sock, LeadData lead)
{
    float v_ego = handler.vEgo;
    cur_state.x_ego = 0.0;
    
    float x_lead, v_lead, a_lead;
    if(lead && lead.status){
        x_lead = lead.dRel;
        v_lead = std::max(float(0.0), lead.vLead;)
        a_lead = lead.aLeadK;

        if(v_lead < 0.1 || -a_lead / 2.0 > v_lead){
            v_lead = 0.0;
            a_lead = 0.0;
        }
        a_lead_tau = lead.aLeadTau;
        new_lead = false;

        if(!prev_lead_status || abs(x_lead - prev_lead_x) > 2.5){
            init_with_simulation(v_mpc, x_lead, v_lead, a_lead, a_lead_tau);
            new_lead = true;
        }

        prev_lead_status = true;
        prev_lead_x = x_lead;
        cur_state.x_l = x_lead;
        cur_state.v_l = v_lead;
    }
    else{
        prev_lead_status = false;
        // Fake a fast lead car, so mpc keeps running
        cur_state.x_l = 50.0;
        cur_state.v_l = v_ego + 10.0;
        a_lead = 0.0;
        a_lead_tau = _LEAD_ACCEL_TAU;
    }

    // Calculate mpc
    float t = sec_since_boot();
    int n_its = run_mpc(cur_state, mpc_solution, a_lead_tau, a_lead);
    int duration = int((sec_since_boot() - t) * 1e9);

    if LOG_MPC:
      send_mpc_solution(livelongitudinalmpc_sock, n_its, duration)

    // Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    v_mpc = mpc_solution.v_ego[1]
    a_mpc = mpc_solution.a_ego[1]
    v_mpc_future = mpc_solution.v_ego[10]

    // Reset if NaN or goes through lead car
    bool crashing = false;
    for(int i = 0; i < 21; ++i){
        if(mpc_solution.x_l[i] - mpc_solution.x_ego[i] < -50){
            crashing = true;
            break;
        }
    }

    bool nans = false;
    for(int i = 0; i < 21; ++i){
        if(std::isnan(mpc_solution.v_ego[i])){
            nans = true;
            break;
        }
    }

    bool backwards = min_array(mpc_solution.v_ego, 21) < -0.01;

    if(((backwards || crashing) && prev_lead_status) || nans){
      if(t > last_cloudlog_t + 5.0){
        last_cloudlog_t = t;
      }

      init(MPC_COST_LONG_TTC, MPC_COST_LONG_DISTANCE, MPC_COST_LONG_ACCELERATION, MPC_COST_LONG_JERK);
      cur_state.v_ego = v_ego;
      cur_state.a_ego = 0.0;
      v_mpc = v_ego;
      a_mpc = handler.aEgo;
      prev_lead_status = false;
    }
}