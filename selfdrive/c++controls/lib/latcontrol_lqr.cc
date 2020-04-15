#include "latcontrol_lqr.h"
#include "utils.h"
#include <cstdio>

float scale_ = 1500.0;
float ki_ = 0.05;
float a[4] = {0., 1., -0.22619643, 1.21822268};
float b[2] = {-1.92006585e-04, 3.95603032e-05};
float c[2] = {1.0, 0.0};
float k[2] = {-110.73572306, 451.22718255};
float l[2] = {0.3233671, 0.3185757};
float dcGain = 0.002237852961363602;
float steerLimitTimer = 0.4;
float steerMaxBP[2] = {4.4444445, 12.5};
float steerMaxV[2] = {1.0, 1.0};

LatControlLQR::LatControlLQR()
{
  angle_steers_des = 0.0;
  scale = scale_;
  ki = ki_;

  for(int i = 0; i < 4; ++i){
    A[i] = a[i];

    if(i < 2){
      B[i] = b[i];
      C[i] = c[i];
      K[i] = k[i];
      L[i] = l[i];
    }
  }

  dc_gain = dcGain;

  x_hat[0] = 0.0;
  x_hat[1] = 0.0;

  i_unwind_rate = 0.3 * DT_CTRL;
  i_rate = 1.0 * DT_CTRL;

  sat_count_rate = 1.0 * DT_CTRL;
  sat_limit = steerLimitTimer;

  lqr_log_active = false;
  lqr_log_steerAngle = 0;
  lqr_log_i = 0;
  lqr_log_output = 0;
  lqr_log_lqrOutput = 0;
  lqr_log_saturated = false;
  reset();
}

LatControlLQR::~LatControlLQR()
{
  printf("-------->LQR: deconstruction \n");
}

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

LatLQRRet LatControlLQR::update(bool active, float v_ego, float angle_steers, 
          float angle_steers_rate, float eps_torque, bool steer_override, 
          bool rate_limited, float path_plan_angleSteers, float path_plan_angleOffset)
{
  LatLQRRet ret;
  float u_lqr;
  float lqr_output;
  float error, i, control;
  bool check_saturation, saturated;
  float steers_max = interp(v_ego, steerMaxBP, steerMaxV, ARRAYSIZE(steerMaxBP));
  float torque_scale = (0.45 + v_ego / 60.0) * (0.45 + v_ego / 60.0);

  angle_steers_des = path_plan_angleSteers - path_plan_angleOffset;
  angle_steers = path_plan_angleOffset;

  float angle_steers_k = C[0] * x_hat[0] + C[1] * x_hat[1];

  float e = angle_steers - angle_steers_k;

  x_hat[0] = (A[0] * x_hat[0] + A[1] * x_hat[1]) + (B[0] * (eps_torque / torque_scale)) + L[0] * e; 
  x_hat[1] = (A[2] * x_hat[0] + A[3] * x_hat[1]) + (B[1] * (eps_torque / torque_scale)) + L[1] * e;
  
  if(v_ego < 0.3 || !active){
    lqr_log_active = false;
    lqr_output = 0.0;
    reset();
  }
  else{
    lqr_log_active = true;

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

  lqr_log_steerAngle = angle_steers_k + path_plan_angleOffset;
  lqr_log_i = i_lqr;
  lqr_log_output = output_steer;
  lqr_log_lqrOutput = lqr_output;
  lqr_log_saturated = saturated;
  // ret.angle_steers_des = angle_steers_des;
  // ret.output_steer = output_steer;
  return ret;
}