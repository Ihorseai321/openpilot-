#include "latcontrol_indi.h"
#include <cmath>

int apply_toyota_steer_torque_limits(float apply_torque, float apply_torque_last, float motor_torque, SteerLimitParams LIMITS)
{  // limits due to comparison of commanded torque VS motor reported torque
  float max_lim = std::min(std::max(motor_torque + LIMITS.STEER_ERROR_MAX, LIMITS.STEER_ERROR_MAX), LIMITS.STEER_MAX);
  float min_lim = std::max(std::min(motor_torque - LIMITS.STEER_ERROR_MAX, -LIMITS.STEER_ERROR_MAX), -LIMITS.STEER_MAX);

  apply_torque = clip(apply_torque, min_lim, max_lim);

  // slow rate if steer torque increases in magnitude
  if(apply_torque_last > 0){
    apply_torque = clip(apply_torque,
                        std::max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP);
  }
  else{
    apply_torque = clip(apply_torque,
                        apply_torque_last - LIMITS.STEER_DELTA_UP,
                        std::min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP));
  }

  return int(std::round(apply_torque));
}

LatControlINDI::LatControlINDI()
{
  LIMIT.STEER_MAX = 1500;
  LIMIT.STEER_DELTA_UP = 10;
  LIMIT.STEER_DELTA_DOWN = 25;
  LIMIT.STEER_ERROR_MAX = 350;

  angle_steers_des = 0.0;
  float tmp[3][3];
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < 3; ++j){
      A[i][j] = 0.0;
      tmp[i][j] = 0.0;  
      A_K[i][j] = 0.0;
      A[i][i] = 1.0;
    }
  }
  A[0][1] = A[1][2] = DT_CTRL;

  for(int i = 0; i < 2; ++i){
    for(int j = 0; j < 3; ++j){
      C[i][j] = 0.0;  
      C[i][i] = 1.0;
    }
  }

  K[0][0] = 7.30262179e-01;
  K[0][1] = 2.07003658e-04;
  K[1][0] = 7.29394177e+00;
  K[1][1] = 1.39159419e-02;
  K[2][0] = 1.71022442e+01;
  K[2][1] = 3.38495381e-02;

  matrix_mul(3, 3, 2, K[0], C[0], tmp[0]);
  matrix_sub(3, 3, A[0], tmp[0], A_K[0]);

  x[0][0] = 0.0;
  x[1][0] = 0.0;
  x[2][0] = 0.0;

  enforce_rate_limit = CP.carName == "toyota";

  RC = CP.lateralTuning_indi_timeConstant;
  G = CP.lateralTuning_indi_actuatorEffectiveness;
  outer_loop_gain = CP.lateralTuning_indi_outerLoopGain;
  inner_loop_gain = CP.lateralTuning_indi_innerLoopGain;
  alpha = 1. - DT_CTRL / (RC + DT_CTRL);

  sat_count_rate = 1.0 * DT_CTRL;
  sat_limit = CP.steerLimitTimer;

  reset();
}

LatControlINDI::~LatControlINDI()
{}

void LatControlINDI::reset()
{
  delayed_output = 0.0;
  output_steer = 0.0;
  sat_count = 0.0;
}

bool LatControlINDI::_check_saturation(float control, bool check_saturation, float limit)
{
    bool saturated = _fabs(control) == limit;
    if(saturated && check_saturation){
        sat_count += sat_count_rate;
    }
    else{
        sat_count -= sat_count_rate;
    }

    sat_count = clip(sat_count, 0.0, 1.0);

    return sat_count > sat_limit;
}

LatINDIRet LatControlINDI::update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, CHandler chandler)
{
  LatINDIRet ret;
  float y[2][1];
  float steers_des = 0.0;
  float rate_des = 0.0;
  float rate_sp = 0.0;
  float accel_sp = 0.0;
  float accel_error = 0.0;
  float g_inv = 0.0;
  float delta_u = 0.0;

  float steer_max = 0.0;
  float new_output_steer_cmd = 0.0;
  float prev_output_steer_cmd = 0.0;

  bool check_saturation = false;

  y[0][0] = angle_steers * DEG_TO_RAD;
  y[1][0] = angle_steers_rate * DEG_TO_RAD;
  float tmp[3][1] = {{0.0}, {0.0}, {0.0}};
  float tmp2[3][1] = {{0.0}, {0.0}, {0.0}};
  matrix_mul(3, 1, 3, A_K[0], x[0], tmp[0]);
  matrix_mul(3, 1, 2, K[0], y[0], tmp2[0]);
  
  for(int i = 0; i < 3; ++i){
    x[i][0] = tmp2[i][0] + tmp[i][0];
  }
  
  ret.indi_log.steerAngle = x[0][0] * RAD_TO_DEG;
  ret.indi_log.steerRate = x[1][0] * RAD_TO_DEG;
  ret.indi_log.steerAccel = x[2][0] * RAD_TO_DEG;

  if(v_ego < 0.3 || !active){
    ret.indi_log.active = false;
    output_steer = 0.0;
    delayed_output = 0.0;
  }
  else{
    angle_steers_des = chandler.angleSteers;
    rate_steers_des = chandler.rateSteers;

    steers_des = angle_steers_des * DEG_TO_RAD;
    rate_des = rate_steers_des * DEG_TO_RAD;

    delayed_output = delayed_output * alpha + output_steer * (1.0 - alpha);

    // Compute acceleration error
    rate_sp = outer_loop_gain * (steers_des - x[0][0]) + rate_des;
    accel_sp = inner_loop_gain * (rate_sp - x[1][0]);
    accel_error = accel_sp - x[2][0];

    // Compute change in actuator
    g_inv = 1. / G;
    delta_u = g_inv * accel_error;

    if(enforce_rate_limit){
      steer_max = LIMITS.STEER_MAX;
      new_output_steer_cmd = steer_max * (delayed_output + delta_u);
      prev_output_steer_cmd = steer_max * output_steer;
      new_output_steer_cmd = apply_toyota_steer_torque_limits(new_output_steer_cmd, prev_output_steer_cmd, prev_output_steer_cmd, LIMITS);
      output_steer = new_output_steer_cmd / steer_max;
    }
    else{
      output_steer = delayed_output + delta_u;
    }

    steer_max = interp(v_ego, CP.steerMaxBP, CP.steerMaxV, ARRAYSIZE(steerMaxBP));
    output_steer = clip(output_steer, -steer_max, steer_max);
    
    ret.indi_log.active = true;
    ret.indi_log.rateSetPoint = rate_sp;
    ret.indi_log.accelSetPoint = accel_sp;
    ret.indi_log.accelError = accel_error;
    ret.indi_log.delayedOutput = delayed_output;
    ret.indi_log.delta = delta_u;
    ret.indi_log.output = output_steer;

    check_saturation = v_ego > 10. && !rate_limited && !steer_override; 
    ret.indi_log.saturated = _check_saturation(output_steer, check_saturation, steers_max);

  }
  ret.output_steer = output_steer;
  ret.angle_steers_des;
  return ret;
}