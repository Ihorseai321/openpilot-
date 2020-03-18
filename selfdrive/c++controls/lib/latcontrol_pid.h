#ifndef LATCONTROL_PID_
#define LATCONTROL_PID_
#include "pid.h"
#include "carparams.h"

typedef struct{
  bool active;
  float steerAngle;
  float steerRate;
  float angleError;
  float p;
  float i;
  float f;
  float output;
  bool saturated;
}LateralPIDState;

typedef struct
{
  float output_steer;
  float angle_steers_des;
  LateralPIDState pid_log;
}LatPIDRet;

class LatControlPID
{
public:
  LatControlPID();
  virtual ~LatControlPID();
  void reset();
  LatPIDRet update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, CHandler chandler);
private:
  CarParams CP;
  PIController pid;
  float angle_steers_des;
};

#endif // LATCONTROL_PID_