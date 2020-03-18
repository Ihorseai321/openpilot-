#include "latcontrol_lqr.h"
#include "utils.h"

LatControlLQR::LatControlLQR()
{
  scale = CP.lateralTuning_lqr_scale;
  ki = CP.lateralTuning_lqr_ki;

  for(int i = 0; i < 4; ++i){
    A[i] = CP.lateralTuning_lqr_a[i];

    if(i < 2){
      B = CP.lateralTuning_lqr_b[i];
      C = CP.lateralTuning_lqr_c[i];
      K = CP.lateralTuning_lqr_k[i];
      L = CP.lateralTuning_lqr_l[i];
    }
  }

  dc_gain = CP.lateralTuning_lqr_dcGain;

  x_hat[0] = 0.0;
  x_hat[1] = 0.0;

  i_unwind_rate = 0.3 * DT_CTRL;
  i_rate = 1.0 * DT_CTRL;

  sat_count_rate = 1.0 * DT_CTRL;
  sat_limit = CP.steerLimitTimer;

  reset();
}

LatControlLQR::~LatControlLQR()
{}

void LatControlLQR::reset()
{
  i_lqr = 0.0;
  output_steer = 0.0;
  sat_count = 0.0;
}

bool LatControlLQR::_check_saturation(float control, bool check_saturation, float limit)
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

LatLQRRet LatControlLQR::update(bool active, float v_ego, float angle_steers, float angle_steers_rate, float eps_torque, bool steer_override, bool rate_limited, CHandler chandler)
{
  LatLQRRet ret;
  float u_lqr;
  float lqr_output;
  float error, i, control;
  bool check_saturation, saturated;
  float steer_max = interp(v_ego, CP.steerMaxBP, CP.steerMaxV, ARRAYSIZE(steerMaxBP));
  float torque_scale = (0.45 + v_ego / 60.0) * (0.45 + v_ego / 60.0);

  angle_steers_des = chandler.angleSteers - chandler.angleOffset;
  float angle_steers = chandler.angleOffset;

  float angle_steers_k = C[0] * x_hat[0] + C[1] * x_hat[1];

  float e = angle_steers - angle_steers_k;

  x_hat[0] = (A[0] * x_hat[0] + A[1] * x_hat[1]) + (B[0] * (eps_torque / torque_scale)) + L[0] * e; 
  x_hat[1] = (A[2] * x_hat[0] + A[3] * x_hat[1]) + (B[1] * (eps_torque / torque_scale)) + L[1] * e;
  
  if(v_ego < 0.3 || !active){
    lqr_log.active = false;
    lqr_output = 0.0;
    reset();
  }
  else{
    ret.lqr_log.active = true;

    // LQR
    u_lqr = angle_steers_des / dc_gain - (K[0] * x_hat[0] + K[1] * x_hat[1]);
    lqr_output = torque_scale * u_lqr / scale;
    // Integrator
    if(steer_override){
      i_lqr -= i_unwind_rate * sign(i_lqr);
    }
    else{
      error = angle_steers_des - angle_steers_k;
      i = i_lqr + ki * i_rate * error;
      control = lqr_output + i;

      if((error >= 0 && (control <= steers_max || i < 0.0)) || (error <= 0 && (control >= -steers_max || i > 0.0))){
        i_lqr = i;
      }
    }

    output_steer = lqr_output + i_lqr;
    output_steer = clip(output_steer, -steers_max, steers_max);
  }
  
  check_saturation = (v_ego > 10) && !rate_limited && !steer_override;
  saturated = _check_saturation(output_steer, check_saturation, steers_max);

  ret.lqr_log.steerAngle = angle_steers_k + chandler.angleOffset;
  ret.lqr_log.i = i_lqr;
  ret.lqr_log.output = output_steer;
  ret.lqr_log.lqrOutput = lqr_output;
  ret.lqr_log.saturated = saturated;
  
  return ret;
}