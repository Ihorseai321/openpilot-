#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <cmath>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "handler.h"
#include "vehicle_model.h"
#include "lateral_mpc/lateral_mpc.h"
#include "lane_planner.h"
#include "config.h"
#include "carparams.h"

#define DT_MDL 0.05
#define MPH_TO_MS 0.44704
#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.29577957

LOG_MPC = std::getenv('LOG_MPC') ? true : false;

float LANE_CHANGE_SPEED_MIN = 45 * MPH_TO_MS;
float LANE_CHANGE_TIME_MAX = 10.0;

int DESIRES[3][4] = {{0, 0, 0, 0}, {0, 0, 3, 3}, {0, 0, 4, 4}};

float MPC_COST_LAT_PATH = 1.0;
float MPC_COST_LAT_LANE = 3.0;
float MPC_COST_LAT_HEADING = 1.0;
float MPC_COST_LAT_STEER_RATE = 1.0;

class PathPlanner
{
public:
	PathPlanner();
	virtual ~PathPlanner();
	void update(Handler handler, PubSocket *pathplan_sock, PubSocket *livempc_sock, VehicleModel VM);
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
};

#endif //PATHPLANNER_H_