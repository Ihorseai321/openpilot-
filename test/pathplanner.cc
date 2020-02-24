#include "pathplanner.h"
#include "utils.h"

PathPlanner::PathPlanner()
{
    MPC_COST_LAT_PATH = 1.0;
    MPC_COST_LAT_LANE = 3.0;
    MPC_COST_LAT_HEADING = 1.0;
    MPC_COST_LAT_STEER_RATE = 1.0;
    last_cloudlog_t = 0;
    steer_rate_cost = CP.steerRateCost;

    setup_mpc();
    solution_invalid_cnt = 0;
    path_offset_i = 0.0;
    lane_change_state = 0;
    lane_change_timer = 0.0;
    prev_one_blinker = False;
}

PathPlanner::~PathPlanner()
{
    
}

void PathPlanner::setup_mpc()
{
    init(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);

    cur_state.x = 0.0;
    cur_state.y = 0.0;
    cur_state.psi = 0.0;
    cur_state.delta = 0.0;

    angle_steers_des = 0.0;
    angle_steers_des_mpc = 0.0;
    angle_steers_des_prev = 0.0;
    angle_steers_des_time = 0.0;
}

state_t PathPlanner::calc_states_after_delay(state_t states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay)
{
    states.x = v_ego * delay;
    states.psi = v_ego * curvature_factor * steer_angle * Conversions::DEG_TO_RAD / steer_ratio * delay;
	return states;
}

void PathPlanner::update(Handler handler, PubSocket *pathplan_sock, PubSocket *livempc_sock, VehicleModel VM)
{
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
    int lane_change_direction = 0; //0 none, 1 left, 2 right
    bool one_blinker = (handler.leftBlinker != handler.rightBlinker);
    bool below_lane_change_speed = (v_ego < LANE_CHANGE_SPEED_MIN);

    if(!active || lane_change_timer > LANE_CHANGE_TIME_MAX){
      lane_change_state = 0;//0 off, 1 preLaneChange, 2 laneChangeStarting, 3 laneChangeFinishing
    }
    else{
      if(handler.leftBlinker){
        lane_change_direction = 1; //0 none, 1 left, 2 right
      }
      else if(handler.rightBlinker){
        lane_change_direction = 2;
      }

      bool torque_applied = handler.steeringPressed && ((handler.steeringTorque > 0 && lane_change_direction == 1) || (handler.steeringTorque < 0 && lane_change_direction == 2));

      float lane_change_prob = LP.l_lane_change_prob + LP.r_lane_change_prob;

      // State transitions
      // off
      if(lane_change_state == 0 && one_blinker && (!prev_one_blinker) && !below_lane_change_speed){
        lane_change_state = 1;
      }
      // pre
      else if(lane_change_state == 1){
        if(!one_blinker || below_lane_change_speed){
          lane_change_state = 0;
        }
        else if(torque_applied){
          lane_change_state = 2;
        }
      }
      // starting
      else if(lane_change_state == 2 && lane_change_prob > 0.5){
        lane_change_state = 3;
      }

      // finishing
      else if(lane_change_state == 3 && lane_change_prob < 0.2){
        if(one_blinker){
          lane_change_state = 1;
        }
        else{
          lane_change_state = 0;
        }
      }

    }

    if(lane_change_state == 0 || lane_change_state == 1){
      lane_change_timer = 0.0;
    }
    else{
      lane_change_timer += DT_MDL;
    }

    prev_one_blinker = one_blinker;

    desire = DESIRES[lane_change_direction][lane_change_state];

    // Turn off lanes during lane change
    if(desire == 4 || desire == 3){
      LP.l_prob = 0.0;
      LP.r_prob = 0.0;
      init_weights(MPC_COST_LAT_PATH / 10.0, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);
    }
    else{
      init_weights(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, steer_rate_cost);
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
    cur_state = calc_states_after_delay(cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay);
    
    v_ego_mpc = v_ego > 5.0 ? v_ego : 5.0;  // avoid mpc roughness due to low speed
    run_mpc(cur_state, mpc_solution, LP.l_poly, LP.r_poly, LP.d_poly, LP.l_prob, LP.r_prob, curvature_factor, v_ego_mpc, LP.lane_width);
    
    // reset to current steer angle if not active or overriding
    if(active){
      float delta_desired = mpc_solution.delta[1];
      float rate_desired = mpc_solution.rate[0] * VM.sR * RAD_TO_DEG;
    }
    else{
      float delta_desired = (angle_steers - angle_offset) * DEG_TO_RAD / VM.sR;
      float rate_desired = 0.0;
    }

    cur_state.delta = delta_desired;

    angle_steers_des_mpc = (delta_desired * VM.sR) * RAD_TO_DEG + angle_offset;

    //  Check for infeasable MPC solution
    bool mpc_nans; // = any(math.isnan(x) for x in mpc_solution.delta)
    
    for(int i = 0; i < 21; ++i){
      mpc_nans = isnan(mpc_solution.delta[i]);
      if(mpc_nans){
        break;
      }
    }
    double t = sec_since_boot();
    if(mpc_nans){
      init(MPC_COST_LAT_PATH, MPC_COST_LAT_LANE, MPC_COST_LAT_HEADING, CP.steerRateCost);
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
    plan_send.setDpoly(d_Poly);
    plan_send.setLpoly(l_Poly);
    plan_send.setRpoly(r_Poly);
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

    delete pp_msg;

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

      delete mpc_msg;
    }

}