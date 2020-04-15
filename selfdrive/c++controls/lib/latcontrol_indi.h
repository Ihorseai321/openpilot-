#ifndef LATCONTROL_INDI_H_
#define LATCONTROL_INDI_H_
#include "carparams.h"
#include "utils.h"


typedef struct{
  float STEER_MAX;
  float STEER_DELTA_UP;       // 1.5s time to peak torque
  float STEER_DELTA_DOWN;     // always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  float STEER_ERROR_MAX;    // max delta between torque cmd and torque motor
}SteerLimitParams; 
  
typedef struct{
  bool active;
  float steerAngle;
  float steerRate;
  float steerAccel;
  float rateSetPoint;
  float accelSetPoint;
  float accelError;
  float delayedOutput;
  float delta;
  float output;
  bool saturated;
}LateralINDIState;

typedef struct
{
  float output_steer;
  float angle_steers_des;
  LateralINDIState indi_log;
}LatINDIRet;

int apply_toyota_steer_torque_limits(float apply_torque, float apply_torque_last, float motor_torque, SteerLimitParams LIMITS);


class LatControlINDI
{
public:
  LatControlINDI();
  ~LatControlINDI();
  void reset();
  LatINDIRet update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, float angleSteers, float rateSteers);

  float angle_steers_des;
  float rate_steers_des;

private:
  bool _check_saturation(float control, bool check_saturation, float limit);

  CarParams CP;
  SteerLimitParams LIMITS;
  float A[3][3];
  float C[2][3];
  float K[3][2];
  float A_K[3][3];
  float x[3][1];
  bool enforce_rate_limit;
  float RC;
  float G;
  float outer_loop_gain;
  float inner_loop_gain;
  float alpha;

  float sat_count_rate;
  float sat_limit;

  float delayed_output;
  float output_steer;
  float sat_count;
};
#endif // LATCONTROL_INDI_H_
