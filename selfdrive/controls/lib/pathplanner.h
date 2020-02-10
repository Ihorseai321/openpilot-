#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <math>
#include "lateral_mpc/lateral_mpc.h"


class PathPlanner
{
public:
	PathPlanner(CarParams CP);
	virtual ~PathPlanner();
	void update();
private:
	state_t calc_states_after_delay(state_t states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay);
	void setup_mpc();
	CarParams CP;
	LanePlanner LP;

    float last_cloudlog_t;
    float steer_rate_cost;

    int solution_invalid_cnt;
    float path_offset_i;
    int lane_change_state;
    float lane_change_timer;
    bool prev_one_blinker;

    log_t mpc_solution;
    state_t cur_state;
    float angle_steers_des;
    float angle_steers_des_mpc;
    float angle_steers_des_prev;
    float angle_steers_des_time;

    float MPC_COST_LAT_PATH;
    float MPC_COST_LAT_LANE;
    float MPC_COST_LAT_HEADING;
    float MPC_COST_LAT_STEER_RATE;
};

#endif //PATHPLANNER_H_