#ifndef LATCONTROL_PID_
#define LATCONTROL_PID_
#include "pid.h"
#include "carparams.h"
#include "ctl_handler.h"
#include "utils.h"

class LatControlPID
{
public:
  LatControlPID();
  virtual ~LatControlPID();
  void reset();
  LatPIDRet update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, CHandler chandler);
  float angle_steers_des;
private:
  CarParams CP;
  PIController pid;
};

#endif // LATCONTROL_PID_