#include "fcw.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

FCWChecker::FCWChecker()
{
    reset_lead(0.0);
    common_counters[0] = 0.0;
    common_counters[1] = 0.0;
}

FCWChecker::~FCWChecker()
{}

void FCWChecker::reset_lead(float cur_time)
{
    last_fcw_a = 0.0;
    v_lead_max = 0.0;
    lead_seen_t = cur_time;
    last_fcw_time = 0.0;
    last_min_a = 0.0;

    for(int i = 0; i < 6; ++i){
        counters[i] = 0.0;
    }
}

float FCWChecker::calc_ttc(float v_ego, float a_ego, float x_lead, float v_lead, float a_lead)
{
    float ttc, max_ttc = 5.0;

    float v_rel = v_ego - v_lead;
    float a_rel = a_ego - a_lead;

    // assuming that closing gap ARel comes from lead vehicle decel,
    // then limit ARel so that v_lead will get to zero in no sooner than t_decel.
    // This helps underweighting ARel when v_lead is close to zero.
    float t_decel = 2.0;
    a_rel = std::min(a_rel, v_lead / t_decel);

    // delta of the quadratic equation to solve for ttc
    float delta = v_rel * v_rel + 2 * x_lead * a_rel;

    // assign an arbitrary high ttc value if there is no solution to ttc
    if(delta < 0.1 || (sqrt(delta) + v_rel < 0.1)){
      ttc = max_ttc;
    }
    else{
      ttc = std::min(float(2 * x_lead / (sqrt(delta) + v_rel)), max_ttc);
    }
    return ttc;
}

bool FCWChecker::update(long_log_t mpc_solution, float cur_time, bool active, float v_ego, float a_ego, float x_lead, float v_lead, float a_lead, float y_lead, float vlat_lead, bool fcw_lead, bool blinkers)
{
    float _FCW_A_ACT_V[2] = {-3.0, -2.0};
    float _FCW_A_ACT_BP[2] = {0.0, 30.0};
    float ttc = 0.0, a_thr = 0.0, a_delta = 0.0;
    bool future_fcw_allowed = true, future_fcw = false;
    float mpc_solution_a[21] = {0};

    for(int i = 0; i < 21; ++i){
        mpc_solution_a[i] = mpc_solution.a_ego[i];
    }

    last_min_a  = min_array(mpc_solution_a, 21);
    v_lead_max = std::max(v_lead_max, v_lead);

    common_counters[0] = blinkers ? common_counters[0] + 10.0 / (20 * 3.0) : 0;
    common_counters[1] = v_ego > 5.0 ? common_counters[1] + 1 : 0;

    if(fcw_lead){
      ttc = calc_ttc(v_ego, a_ego, x_lead, v_lead, a_lead);
      counters[0] = ttc < 2.5 ? counters[0] + 1 : 0;
      counters[1] = v_lead_max > 2.5 ? counters[1] + 1 : 0;
      counters[2] = v_ego > v_lead ? counters[2] + 1 : 0;
      counters[3] = counters[3] + 0.33;
      counters[4] = std::abs(y_lead) < 1.0 ? counters[4] + 1 : 0;
      counters[5] = std::abs(vlat_lead) < 0.4 ? counters[5] + 1 : 0;

      a_thr = interp(v_lead, _FCW_A_ACT_BP, _FCW_A_ACT_V, 2);
      a_delta = min_array(mpc_solution_a, 15) - std::min(float(0.0), a_ego);

      for(int i = 0; i < 6; ++i){
          if(counters[i] < 10){
              future_fcw_allowed = false;
              break;
          }
      }
      
      future_fcw_allowed = future_fcw_allowed && (common_counters[0] >= 10 && common_counters[1] >= 10);
      future_fcw = (last_min_a < -3.0 || a_delta < a_thr) && future_fcw_allowed;

      if(future_fcw && (last_fcw_time + 5.0 < cur_time)){
        last_fcw_time = cur_time;
        last_fcw_a = last_min_a;
        return true;
      }
    }

    return false;
}