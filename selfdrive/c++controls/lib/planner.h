#ifndef PLANNER_H_
#define PLANNER_H_
#include "carparams.h"
#include "fcw.h"
#include "long_mpc.h"
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"



void calc_cruise_accel_limits(float v_ego, bool following, float ret[2][1]);
void limit_accel_in_turns(float v_ego, float angle_steers, float a_target, CarParams CP, float ret[]);

class Planner
{
public:
    Planner();
    virtual ~Planner();
    void choose_solution(float v_cruise_setpoint, bool enabled);
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

    cereal::Plan::LongitudinalPlanSource longitudinalPlanSource;
    float path_x[192];

    bool first_loop;
};
#endif // PLANNER_H_