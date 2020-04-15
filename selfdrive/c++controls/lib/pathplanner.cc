#include "pathplanner.h"
#include "common/timing.h"
#include "lateral_mpc/lateral_mpc.h"

#include <iostream>
using namespace std;

PathPlanner::PathPlanner()
{
    last_cloudlog_t = 0;
    steer_rate_cost = CP.steerRateCost;

    setup_mpc();
    solution_invalid_cnt = 0;
    path_offset_i = 0.0;
    lane_change_state = cereal::PathPlan::LaneChangeState::OFF;
    lane_change_timer = 0.0;
    prev_one_blinker = false;

    MPC_COST_LAT_PATH = 1.0;
    MPC_COST_LAT_LANE = 3.0;
    MPC_COST_LAT_HEADING = 1.0;
    MPC_COST_LAT_STEER_RATE = 1.0;
}

PathPlanner::~PathPlanner()
{
    
}

void PathPlanner::setup_mpc()
{
    lateral_init(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);

    cur_state.x = 0.0;
    cur_state.y = 0.0;
    cur_state.psi = 0.0;
    cur_state.delta = 0.0;

    angle_steers_des = 0.0;
    angle_steers_des_mpc = 0.0;
    angle_steers_des_prev = 0.0;
    angle_steers_des_time = 0.0;
}

void PathPlanner::calc_states_after_delay(lateral_state_t *states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay)
{
    states->x = v_ego * delay;
    states->psi = v_ego * curvature_factor * steer_angle * DEG_TO_RAD / steer_ratio * delay;
}

void PathPlanner::update(Handler handler, PubSocket *pathplan_sock, PubSocket *livempc_sock, VehicleModel VM)
{
    float LANE_CHANGE_SPEED_MIN = 45 * MPH_TO_MS;
    float LANE_CHANGE_TIME_MAX = 10.0;

    int DESIRES[3][4] = {{0, 0, 0, 0}, {0, 0, 3, 3}, {0, 0, 4, 4}};

    float v_ego = handler.vEgo;
    float angle_steers = handler.steeringAngle;
    bool active = handler.active;

    float angle_offset = handler.angleOffset;

    // Run MPC
    angle_steers_des_prev = angle_steers_des_mpc;
    VM.update_params(handler.stiffnessFactor, handler.steerRatio);
    float curvature_factor = VM.curvature_factor(v_ego);

    LP.parse_model(handler);

    // Lane change logic
    cereal::PathPlan::LaneChangeDirection lane_change_direction = cereal::PathPlan::LaneChangeDirection::NONE; //0 none, 1 left, 2 right
    bool one_blinker = (handler.leftBlinker != handler.rightBlinker);
    bool below_lane_change_speed = (v_ego < LANE_CHANGE_SPEED_MIN);

    if(!active || lane_change_timer > LANE_CHANGE_TIME_MAX){
      lane_change_state = cereal::PathPlan::LaneChangeState::OFF;//0 off, 1 preLaneChange, 2 laneChangeStarting, 3 laneChangeFinishing
    }
    else{
      if(handler.leftBlinker){
        lane_change_direction = cereal::PathPlan::LaneChangeDirection::LEFT; //0 none, 1 left, 2 right
      }
      else if(handler.rightBlinker){
        lane_change_direction = cereal::PathPlan::LaneChangeDirection::RIGHT;
      }

      bool torque_applied = handler.steeringPressed && ((handler.steeringTorque > 0 && lane_change_direction == cereal::PathPlan::LaneChangeDirection::LEFT) || (handler.steeringTorque < 0 && lane_change_direction == cereal::PathPlan::LaneChangeDirection::RIGHT));

      float lane_change_prob = LP.l_lane_change_prob + LP.r_lane_change_prob;

      // State transitions
      // off
      if(lane_change_state == cereal::PathPlan::LaneChangeState::OFF && one_blinker && (!prev_one_blinker) && !below_lane_change_speed){
        lane_change_state = cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE;
      }
      // pre
      else if(lane_change_state == cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE){
        if(!one_blinker || below_lane_change_speed){
          lane_change_state = cereal::PathPlan::LaneChangeState::OFF;
        }
        else if(torque_applied){
          lane_change_state = cereal::PathPlan::LaneChangeState::LANE_CHANGE_STARTING;
        }
      }
      // starting
      else if(lane_change_state == cereal::PathPlan::LaneChangeState::LANE_CHANGE_STARTING && lane_change_prob > 0.5){
        lane_change_state = cereal::PathPlan::LaneChangeState::LANE_CHANGE_FINISHING;
      }

      // finishing
      else if(lane_change_state == cereal::PathPlan::LaneChangeState::LANE_CHANGE_FINISHING && lane_change_prob < 0.2){
        if(one_blinker){
          lane_change_state = cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE;
        }
        else{
          lane_change_state = cereal::PathPlan::LaneChangeState::OFF;
        }
      }

    }

    if(lane_change_state == cereal::PathPlan::LaneChangeState::OFF || lane_change_state == cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE){
      lane_change_timer = 0.0;
    }
    else{
      lane_change_timer += DT_MDL;
    }

    prev_one_blinker = one_blinker;
    
    int row, col;
    if(lane_change_direction == cereal::PathPlan::LaneChangeDirection::RIGHT){
      row = 2;
    }
    else if(lane_change_direction == cereal::PathPlan::LaneChangeDirection::LEFT){
      row = 1;
    }
    else{
      row = 0;
    }

    if(lane_change_state == cereal::PathPlan::LaneChangeState::LANE_CHANGE_FINISHING){
      col = 3;
    }
    else if(lane_change_state == cereal::PathPlan::LaneChangeState::LANE_CHANGE_STARTING){
      col = 2;
    }
    else if(lane_change_state == cereal::PathPlan::LaneChangeState::PRE_LANE_CHANGE){
      col = 1;
    }
    else{
      col = 0;
    }

    cereal::PathPlan::Desire desire;
    int desire_num = DESIRES[row][col];
    if(desire_num == 3){
      desire = cereal::PathPlan::Desire::LANE_CHANGE_LEFT;
    }
    else if(desire_num == 4){
      desire = cereal::PathPlan::Desire::LANE_CHANGE_RIGHT;
    }
    else{
      desire = cereal::PathPlan::Desire::NONE;
    }
    

    // Turn off lanes during lane change
    if(desire == cereal::PathPlan::Desire::LANE_CHANGE_RIGHT || desire == cereal::PathPlan::Desire::LANE_CHANGE_LEFT){
      LP.l_prob = 0.0;
      LP.r_prob = 0.0;
      lateral_init_weights(MPC_COST_LAT_PATH / 10.0, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);
    }
    else{
      lateral_init_weights(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);
    }

    LP.update_d_poly(v_ego);


    // TODO: Check for active, override, and saturation
    // if active:
    //   self.path_offset_i += self.LP.d_poly[3] / (60.0 * 20.0)
    //   self.path_offset_i = clip(self.path_offset_i, -0.5,  0.5)
    //   self.LP.d_poly[3] += self.path_offset_i
    // else:
    //   self.path_offset_i = 0.0

    // account for actuation delay
    calc_states_after_delay(&cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay);
    
    float v_ego_mpc = v_ego > 5.0 ? v_ego : 5.0;  // avoid mpc roughness due to low speed
    lateral_run_mpc(&cur_state, &mpc_solution, LP.l_poly, LP.r_poly, LP.d_poly, LP.l_prob, LP.r_prob, curvature_factor, v_ego_mpc, LP.lane_width);
    
    // reset to current steer angle if not active or overriding
    float delta_desired, rate_desired;
    if(active){
      delta_desired = mpc_solution.delta[1];
      rate_desired = mpc_solution.rate[0] * VM.sR * RAD_TO_DEG;
    }
    else{
      delta_desired = (angle_steers - angle_offset) * DEG_TO_RAD / VM.sR;
      rate_desired = 0.0;
    }

    cur_state.delta = delta_desired;

    angle_steers_des_mpc = (delta_desired * VM.sR) * RAD_TO_DEG + angle_offset;

    //  Check for infeasable MPC solution
    bool mpc_nans; // = any(math.isnan(x) for x in mpc_solution.delta)
    
    for(int i = 0; i < 21; ++i){
      mpc_nans = std::isnan(mpc_solution.delta[i]);
      if(mpc_nans){
        break;
      }
    }
    double t = sec_since_boot();
    if(mpc_nans){
      lateral_init(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, CP.steerRateCost);
      cur_state.delta = (angle_steers - angle_offset) * DEG_TO_RAD / VM.sR;
     
      if(t > last_cloudlog_t + 5.0){
        last_cloudlog_t = t;
      }
    }

    if(mpc_solution.cost > 20000.0 || mpc_nans){   // TODO: find a better way to detect when MPC did not converge
      solution_invalid_cnt += 1;
    }
    else{
      solution_invalid_cnt = 0;
    }

    bool plan_solution_valid = solution_invalid_cnt < 2;

    capnp::MallocMessageBuilder pp_msg;
    cereal::Event::Builder event = pp_msg.initRoot<cereal::Event>();
    event.setLogMonoTime(nanos_since_boot());
    auto plan_send = event.initPathPlan();

    kj::ArrayPtr<const float> d_Poly(&LP.d_poly[0], 4);
    kj::ArrayPtr<const float> l_Poly(&LP.l_poly[0], 4);
    kj::ArrayPtr<const float> r_Poly(&LP.r_poly[0], 4);

    plan_send.setLaneWidth(LP.lane_width);
    plan_send.setDPoly(d_Poly);
    plan_send.setLPoly(l_Poly);
    plan_send.setRPoly(r_Poly);
    plan_send.setLProb(LP.l_prob);
    plan_send.setRProb(LP.r_prob);
    
    plan_send.setAngleSteers(angle_steers_des_mpc);
    plan_send.setRateSteers(rate_desired);
    plan_send.setAngleOffset(handler.angleOffsetAverage);
    plan_send.setMpcSolutionValid(plan_solution_valid);
    plan_send.setParamsValid(handler.paramsValid);
    plan_send.setSensorValid(handler.sensorValid);
    plan_send.setPosenetValid(handler.posenetValid);

    plan_send.setDesire(desire);
    plan_send.setLaneChangeState(lane_change_state);
    plan_send.setLaneChangeDirection(lane_change_direction);

    auto words = capnp::messageToFlatArray(pp_msg);
    auto bytes = words.asBytes();
    pathplan_sock->send((char*)bytes.begin(), bytes.size());
    cout << "pathplan PubSocket" << endl;
 
    bool LOG_MPC = std::getenv("LOG_MPC") ? true : false;

    if(LOG_MPC){
      capnp::MallocMessageBuilder mpc_msg;
      cereal::Event::Builder event = mpc_msg.initRoot<cereal::Event>();
      event.setLogMonoTime(nanos_since_boot());
      auto mpc_send = event.initLiveMpc();

      kj::ArrayPtr<const float> x_data(&mpc_solution.x[0], 21);
      kj::ArrayPtr<const float> y_data(&mpc_solution.y[0], 21);
      kj::ArrayPtr<const float> psi_data(&mpc_solution.psi[0], 21);
      kj::ArrayPtr<const float> delta_data(&mpc_solution.delta[0], 21);

      mpc_send.setX(x_data);
      mpc_send.setY(y_data);
      mpc_send.setPsi(psi_data);
      mpc_send.setDelta(delta_data);
      mpc_send.setCost(mpc_solution.cost);

      auto words = capnp::messageToFlatArray(mpc_msg);
      auto bytes = words.asBytes();
      livempc_sock->send((char*)bytes.begin(), bytes.size());

    }

}