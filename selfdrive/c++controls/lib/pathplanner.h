#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <cmath>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "handler.h"
#include "vehicle_model.h"
#include "lane_planner.h"
#include "carparams.h"
#include "mpc_structs.h"


class PathPlanner
{
public:
	PathPlanner();
	virtual ~PathPlanner();
	void update(Handler handler, PubSocket *pathplan_sock, PubSocket *livempc_sock, VehicleModel VM);
private:
    void calc_states_after_delay(lateral_state_t *states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay);
	void setup_mpc();
	CarParams CP;
	LanePlanner LP;

    float last_cloudlog_t;
    float steer_rate_cost;

    int solution_invalid_cnt;
    float path_offset_i;
    cereal::PathPlan::LaneChangeState lane_change_state;
    float lane_change_timer;
    bool prev_one_blinker;

    lateral_log_t mpc_solution;
    lateral_state_t cur_state;
    
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