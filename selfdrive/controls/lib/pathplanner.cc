#include "pathplanner.h"

PathPlanner::PathPlanner(CarParams CP)
{
	this.CP = CP;
	LanePlanner LP = LanePlanner();
	
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
	states.psi = v_ego * curvature_factor * steer_angle * RADIANS_TO_DEGREE / steer_ratio * delay;
	return states;
}

void PathPlanner::update()
{
	
}