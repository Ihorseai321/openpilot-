#include "latcontrol_lqr.h"

extern "C" {
  
  LatControlLQR lqr;
  
  float getAngleSteersDes()
  {
    return lqr.angle_steers_des;
  }

  float getOutputSteer()
  {
    return lqr.output_steer;
  }

  bool get_lqr_log_saturated()
  {
    return lqr.lqr_log_saturated;
  }

  float get_lqr_log_lqrOutput()
  {
    return lqr.lqr_log_lqrOutput;
  }

  float get_lqr_log_output()
  {
    return lqr.lqr_log_output;
  }

  float get_lqr_log_i()
  {
    return lqr.lqr_log_i;
  }

  float get_lqr_log_steerAngle()
  {
    return lqr.lqr_log_steerAngle;
  }

  bool get_lqr_log_active()
  {
    return lqr.lqr_log_active;
  }


  void reset()
  {
    lqr.reset();
  }

  void update(bool active, float v_ego, float angle_steers, 
          float angle_steers_rate, float eps_torque, bool steer_override, 
          bool rate_limited, float path_plan_angleSteers, float path_plan_angleOffset)
  {
    lqr.update(active, v_ego, angle_steers, angle_steers_rate, eps_torque, 
        steer_override, rate_limited, path_plan_angleSteers, path_plan_angleOffset);
  }

}