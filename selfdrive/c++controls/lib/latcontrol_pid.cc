#include "latcontrol_pid.h"
#include "utils.h"
#include "cereal/gen/cpp/car.capnp.h"

LatControlPID::LatControlPID():pid(CP.lateralTuning_pid_kpBP, CP.lateralTuning_pid_kpV,
                                   CP.lateralTuning_pid_kiBP, CP.lateralTuning_pid_kiV, 100, 
                                   CP.steerLimitTimer, false, CP.lateralTuning_pid_kf, 1.0, 0.0)
{
  angle_steers_des = 0.0;
}

LatControlPID::~LatControlPID()
{}

void LatControlPID::reset()
{
  pid.reset();
}

LatPIDRet LatControlPID::update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, CHandler chandler)
{
  LatPIDRet ret;
  ret.pid_log.steerAngle = angle_steers;
  ret.pid_log.steerRate = angle_steers_rate;
  
  float steers_max = 0.0;
  float steer_feedforward = 0.0;
  float deadzone = 0.0;
  bool check_saturation;

  if(v_ego < 0.3 || !active){
    ret.output_steer = 0.0;
    ret.pid_log.active = false;
    pid.reset();
  }
  else{
    angle_steers_des = chandler.angleSteers;
    steers_max = interp(v_ego, CP.steerMaxBP, CP.steerMaxV, ARRAYSIZE(steerMaxBP));
    pid.pos_limit = steers_max;
    pid.neg_limit = -steers_max;
    steer_feedforward = angle_steers_des;

    if (CP._steerControlType == cereal::CarParams::SteerControlType::TORQUE)
    {
      steer_feedforward -= chandler.angleOffset;
      steer_feedforward *= (v_ego * v_ego);
    }

    deadzone = 0.0;
    check_saturation = v_ego > 10 && !rate_limited && !steer_override;
    ret.output_steer = pid.update(angle_steers_des, angle_steers, v_ego, deadzone, 
                                  steer_feedforward, false, check_saturation, steer_override);
    ret.pid_log.active = true;
    ret.pid_log.p = pid.p;
    ret.pid_log.i = pid.i;
    ret.pid_log.f = pid.f;
    ret.pid_log.output = ret.output_steer;
    ret.pid_log.saturated = pid.saturated; 
  }
  ret.angle_steers_des = angle_steers_des;

  return ret;
}