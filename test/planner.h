#ifndef PLANNER_H_
#define PLANNER_H_
#include "carparams.h"

const float DEG_TO_RAD = 0.01745329;
const float MPH_TO_MS = 0.44704;
const float MAX_SPEED = 255.0;
const float KPH_TO_MS = 0.277777778;
const float LON_MPC_STEP = 0.2;  // first step is 0.2s
const float MAX_SPEED_ERROR = 2.0;
const float AWARENESS_DECEL = -0.2;     // car smoothly decel at .2m/s^2 when user is distracted
const float MIN_CAN_SPEED = 0.3;
// lookup tables VS speed to determine min and max accels in cruise
// make sure these accelerations are smaller than mpc limits
const float _A_CRUISE_MIN_V[5] = {-1.0, -0.8, -0.67, -0.5, -0.30};
const float _A_CRUISE_MIN_BP[5] = {0.0, 5.0, 10.0, 20.0, 40.0};

// need fast accel at very low speed for stop and go
// make sure these accelerations are smaller than mpc limits
const float _A_CRUISE_MAX_V[4] = {1.2, 1.2, 0.65, 0.4};
const float _A_CRUISE_MAX_V_FOLLOWING[4] = {1.6, 1.6, 0.65, 0.4};
const float _A_CRUISE_MAX_BP[4] = {0.0,  6.4, 22.5, 40.0};

// Lookup table for turns
const float _A_TOTAL_MAX_V[2] = {1.7, 3.2};
const float _A_TOTAL_MAX_BP[2] = {20.0, 40.0};

// 75th percentile
const int SPEED_PERCENTILE_IDX = 7;

void calc_cruise_accel_limits(float v_ego, bool following, float ret[2][1]);
void limit_accel_in_turns(float v_ego, float angle_steers, float a_target, CarParams CP, float ret[]);

class Planner
{
public:
    Planner();
    virtual ~Planner();
    void choose_solution(v_cruise_setpoint, enabled);
    void update(Handler handler, PubSocket *plan_sock, PubSocket *livelongitudinalmpc_sock);

private:
    CarParams CP;
    FCWChecker fcw_checker;

    LongitudinalMpc mpc1;
    LongitudinalMpc mpc2;

    float v_acc_start;
    float a_acc_start;

    float v_acc;
    float v_acc_future;
    float a_acc;
    float v_cruise;
    float a_cruise;
    float v_model;
    float a_model;

    std::string longitudinalPlanSource;
    float path_x[192];

    bool first_loop;
};
#endif // PLANNER_H_