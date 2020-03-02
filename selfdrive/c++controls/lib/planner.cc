#include <cmath>
#include <algorithm>
#include "planner.h"
#include "utils.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "speed_smoother.h"

#include <iostream>
using namespace std;

const float MAX_SPEED = 255.0;
const float KPH_TO_MS = 0.277777778;
const float LON_MPC_STEP = 0.2;  // first step is 0.2s
const float MAX_SPEED_ERROR = 2.0;
const float AWARENESS_DECEL = -0.2;     // car smoothly decel at .2m/s^2 when user is distracted
const float MIN_CAN_SPEED = 0.3;
// lookup tables VS speed to determine min and max accels in cruise
// make sure these accelerations are smaller than mpc limits
float _A_CRUISE_MIN_V[5] = {-1.0, -0.8, -0.67, -0.5, -0.30};
float _A_CRUISE_MIN_BP[5] = {0.0, 5.0, 10.0, 20.0, 40.0};

// need fast accel at very low speed for stop and go
// make sure these accelerations are smaller than mpc limits
float _A_CRUISE_MAX_V[4] = {1.2, 1.2, 0.65, 0.4};
float _A_CRUISE_MAX_V_FOLLOWING[4] = {1.6, 1.6, 0.65, 0.4};
float _A_CRUISE_MAX_BP[4] = {0.0,  6.4, 22.5, 40.0};

// Lookup table for turns
float _A_TOTAL_MAX_V[2] = {1.7, 3.2};
float _A_TOTAL_MAX_BP[2] = {20.0, 40.0};

// 75th percentile
const int SPEED_PERCENTILE_IDX = 7;

void calc_cruise_accel_limits(float v_ego, bool following, float ret[2])
{
    float a_cruise_max = 0.0, a_cruise_min = 0.0;
    a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V, 5);

    if(following){
        a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V_FOLLOWING, 4);
    }
    else{
        a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V, 4);
    }

    ret[0] = a_cruise_min;
    ret[1] = a_cruise_max;
}

void limit_accel_in_turns(float v_ego, float angle_steers, float a_target[], CarParams CP, float ret[])
{
    float a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V, 2);
    float a_y = v_ego * v_ego * angle_steers * DEG_TO_RAD / (CP.steerRatio * CP.wheelbase);
    float a_x_allowed = sqrt(std::max(a_total_max * a_total_max - a_y * a_y, float(0.0)));

    ret[0] = a_target[0];
    ret[1] = std::min(a_target[1], a_x_allowed);
}

Planner::Planner():mpc1(1), mpc2(2)
{
    v_acc_start = 0.0;
    a_acc_start = 0.0;

    v_acc = 0.0;
    v_acc_future = 0.0;
    a_acc = 0.0;
    v_cruise = 0.0;
    a_cruise = 0.0;
    v_model = 0.0;
    a_model = 0.0;

    longitudinalPlanSource = cereal::Plan::LongitudinalPlanSource::CRUISE;
    for(int i = 0; i < 192; ++i){
        path_x[i] = float(i);
    }

    first_loop = true;
}

Planner::~Planner()
{}

void Planner::choose_solution(float v_cruise_setpoint, bool enabled)
{
    float min_v_future;
    if(enabled){
        cereal::Plan::LongitudinalPlanSource PlanSource[4] = {cereal::Plan::LongitudinalPlanSource::MODEL, cereal::Plan::LongitudinalPlanSource::CRUISE, cereal::Plan::LongitudinalPlanSource::MPC1, cereal::Plan::LongitudinalPlanSource::MPC2};
        int slow1, slow2, slowest;
        float solutions[4] = {100.0, 100.0, 100.0, 100.0}; // 0 "model", 1 "cruise", 2 "mpc1", 3 "mpc2"
        solutions[0] = v_model;
        solutions[1] = v_cruise;

        if(mpc1.prev_lead_status){
            solutions[2] = mpc1.v_mpc;
        }
        if(mpc2.prev_lead_status){
            solutions[3] = mpc2.v_mpc;
        }

        slow1 = solutions[0] < solutions[1] ? 0 : 1;
        slow2 = solutions[2] < solutions[3] ? 2 : 3;
        slowest = solutions[slow1] < solutions[slow2] ? slow1 : slow2;

        longitudinalPlanSource = PlanSource[slowest];
        // Choose lowest of MPC and cruise
        if(slowest == 2){
            v_acc = mpc1.v_mpc;
            a_acc = mpc1.a_mpc;
        }
        else if(slowest == 3){
            v_acc = mpc2.v_mpc;
            a_acc = mpc2.a_mpc;
        }
        else if(slowest == 1){
            v_acc = v_cruise;
            a_acc = a_cruise;
        }
        else if(slowest == 0){
            v_acc = v_model;
            a_acc = a_model;
        }
    }

    min_v_future = mpc1.v_mpc_future < mpc2.v_mpc_future ? mpc1.v_mpc_future : mpc2.v_mpc_future;
    v_acc_future = min_v_future < v_cruise_setpoint ? min_v_future : v_cruise_setpoint;

}

void Planner::update(Handler handler, PubSocket *plan_sock, PubSocket *livelongitudinalmpc_sock)
{
    float accel_limits[2] = {0.0, 0.0};
    float jerk_limits[2] = {0.0, 0.0};
    float accel_limits_turns[2] = {0.0, 0.0};
    float ret_cruise[2] = {0.0, 0.0};
    float ret_model[2] = {0.0, 0.0};
    float path[4] = {0.0, 0.0, 0.0, 0.0};
    float y_p, y_pp, a_y_max, model_speed;
    float curv[192] = {0.0};
    float min_v_curcature = 10000.0, v_curvature = 0.0;
    

    double cur_time = sec_since_boot();
    float v_ego = handler.vEgo;

    cereal::ControlsState::LongControlState long_control_state = handler.long_control_state;
    float v_cruise_kph = handler.v_cruise_kph;
    bool force_slow_decel = handler.force_slow_decel;
    float v_cruise_setpoint = v_cruise_kph * KPH_TO_MS;

    struct LeadData lead_1;
    struct LeadData lead_2;
    lead_1 = handler.lead_1;
    lead_2 = handler.lead_2;
    //LongControlState: 0 off, 1 pid, 2 stopping, 3 starting
    bool enabled = (long_control_state == cereal::ControlsState::LongControlState::PID) || (long_control_state == cereal::ControlsState::LongControlState::STOPPING);
    bool following = lead_1.status && lead_1.dRel < 45.0 && lead_1.vLeadK > v_ego && lead_1.aLeadK > 0.0;

    if(handler.p_has_poly){
        for(int i = 0; i < 4; ++i){
            path[i] = handler.p_poly[i];
        }
      
        // Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
        // y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
        // k = y'' / (1 + y'^2)^1.5
        // TODO: compute max speed without using a list of points and without numpy
        
        for(int j = 0; j < 192; ++j){
            y_p = 3 * path[0] * path_x[j] * path_x[j] + 2 * path[1] * path_x[j] + path[2];
            y_pp = 6 * path[0] * path_x[j] + 2 * path[1];
            curv[j] = y_pp / pow((1.0 + y_p * y_p), 1.5);
        }

        a_y_max = 2.975 - v_ego * 0.0375;  // ~1.85 @ 75mph, ~2.6 @ 25mph
        
        
        for(int i = 0; i < 192; ++i){
            if(std::abs(curv[i]) < 1e-4){
                v_curvature = sqrt(a_y_max / 1e-4);
            }
            else{
                v_curvature = sqrt(a_y_max / std::abs(curv[i]));
            }
            min_v_curcature = min_v_curcature < v_curvature ? min_v_curcature : v_curvature;
        }
        model_speed = min_v_curcature;
        model_speed = std::max(float(20.0 * MPH_TO_MS), model_speed); // Don't slow down below 20mph
    }
    else{
        model_speed = MAX_SPEED;
    }

    // Calculate speed for normal cruise control
    if(enabled && !first_loop){
      calc_cruise_accel_limits(v_ego, following, accel_limits);
      
      jerk_limits[0] = std::min(float(-0.1), accel_limits[0]);
      jerk_limits[1] = std::max(float(0.1), accel_limits[1]); // TODO: make a separate lookup for jerk tuning

      limit_accel_in_turns(v_ego, handler.steeringAngle, accel_limits, CP, accel_limits_turns);

      if(force_slow_decel){
        // if required so, force a smooth deceleration
        accel_limits_turns[1] = std::min(accel_limits_turns[1], AWARENESS_DECEL);
        accel_limits_turns[0] = std::min(accel_limits_turns[0], accel_limits_turns[1]);

      }

      speed_smoother(v_acc_start, a_acc_start, v_cruise_setpoint, accel_limits_turns[1], 
                     accel_limits_turns[0], jerk_limits[1], jerk_limits[0], LON_MPC_STEP, ret_cruise);
      
      v_cruise = ret_cruise[0];
      a_cruise = ret_cruise[1];

      speed_smoother(v_acc_start, a_acc_start, model_speed, 2*accel_limits[1],
                     accel_limits[0], 2*jerk_limits[1], jerk_limits[0], LON_MPC_STEP, ret_model);

      v_model = ret_model[0];
      a_model = ret_model[1];
      // cruise speed can't be negative even is user is distracted
      v_cruise = std::max(v_cruise, float(0.0));
    }
    else{
      bool starting = (long_control_state == cereal::ControlsState::LongControlState::STARTING);
      float a_ego = std::min(handler.aEgo, float(0.0));
      float reset_speed = starting ? MIN_CAN_SPEED : v_ego;
      float reset_accel = starting ? CP.startAccel : a_ego;
      v_acc = reset_speed;
      a_acc = reset_accel;
      v_acc_start = reset_speed;
      a_acc_start = reset_accel;
      v_cruise = reset_speed;
      a_cruise = reset_accel;
    }

    mpc1.set_cur_state(v_acc_start, a_acc_start);
    mpc2.set_cur_state(v_acc_start, a_acc_start);

    mpc1.update(handler, livelongitudinalmpc_sock, lead_1);
    mpc2.update(handler, livelongitudinalmpc_sock, lead_2);

    choose_solution(v_cruise_setpoint, enabled);

    // determine fcw
    if(mpc1.new_lead){
      fcw_checker.reset_lead(cur_time);
    }

    bool blinkers = handler.leftBlinker || handler.rightBlinker;
    bool fcw = fcw_checker.update(mpc1.mpc_solution, cur_time, handler.active, v_ego, handler.aEgo,
                             lead_1.dRel, lead_1.vLead, lead_1.aLeadK, lead_1.yRel, lead_1.vLat,
                             lead_1.fcw, blinkers) && !handler.brakePressed;
    // if(fcw){
    //   LOGW("FCW triggered %s", fcw_checker.counters);
    // }

    bool radar_dead = false;

    bool radar_fault = false;
    bool radar_can_error = false;

    // **** send the plan ****
    capnp::MallocMessageBuilder pp_msg;
    cereal::Event::Builder event = pp_msg.initRoot<cereal::Event>();
    unsigned long plan_monotime = nanos_since_boot();
    event.setLogMonoTime(plan_monotime);
    auto plan_send = event.initPlan();

    plan_send.setMdMonoTime(handler.model_time);
    plan_send.setRadarStateMonoTime(handler.radar_state_time);
    
    plan_send.setVCruise(v_cruise);
    plan_send.setACruise(a_cruise);
    plan_send.setVStart(v_acc_start);
    plan_send.setAStart(a_acc_start);
    plan_send.setVTarget(v_acc);
    plan_send.setATarget(a_acc);
    plan_send.setVTargetFuture(v_acc_future);
    plan_send.setHasLead(mpc1.prev_lead_status);
    plan_send.setLongitudinalPlanSource(longitudinalPlanSource);

    bool radar_valid = !(radar_dead || radar_fault);
    plan_send.setRadarValid(radar_valid);
    plan_send.setRadarCanError(radar_can_error);

    plan_send.setProcessingDelay((plan_monotime / 1e9) - handler.radar_rcv_time);

    // Send out fcw
    plan_send.setFcw(fcw);

    auto words = capnp::messageToFlatArray(pp_msg);
    auto bytes = words.asBytes();
    plan_sock->send((char*)bytes.begin(), bytes.size());
    cout << "plan PubSocket" << endl;
    /*
    

    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'radarState'])

    plan_send.plan.mdMonoTime = sm.logMonoTime['model']
    plan_send.plan.radarStateMonoTime = sm.logMonoTime['radarState']

    // longitudal plan
    plan_send.plan.vCruise = float(self.v_cruise)
    plan_send.plan.aCruise = float(self.a_cruise)
    plan_send.plan.vStart = float(self.v_acc_start)
    plan_send.plan.aStart = float(self.a_acc_start)
    plan_send.plan.vTarget = float(self.v_acc)
    plan_send.plan.aTarget = float(self.a_acc)
    plan_send.plan.vTargetFuture = float(self.v_acc_future)
    plan_send.plan.hasLead = self.mpc1.prev_lead_status
    plan_send.plan.longitudinalPlanSource = self.longitudinalPlanSource

    radar_valid = not (radar_dead or radar_fault)
    plan_send.plan.radarValid = bool(radar_valid)
    plan_send.plan.radarCanError = bool(radar_can_error)

    plan_send.plan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.rcv_time['radarState']

    // Send out fcw
    plan_send.plan.fcw = fcw
    print("----------------planner.py : %s", plan_send.plan.fcw)
    pm.send('plan', plan_send)
    */

    // Interpolate 0.05 seconds and save as starting point for next iteration
    float a_acc_sol = a_acc_start + (CP.radarTimeStep / LON_MPC_STEP) * (a_acc - a_acc_start);
    float v_acc_sol = v_acc_start + CP.radarTimeStep * (a_acc_sol + a_acc_start) / 2.0;
    v_acc_start = v_acc_sol;
    a_acc_start = a_acc_sol;

    first_loop = false;

}
