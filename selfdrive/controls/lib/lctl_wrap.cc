#include "longcontrol.h"

extern "C" {
  
  LongControl lctl;
  
  float getFinalGas()
  {
    return lctl.final_gas;
  }

  float getFinalBrake()
  {
    return lctl.final_brake;
  }

  void reset(float v_pid)
  {
    lctl.reset(v_pid);
  }

  void update(bool active, float v_ego, bool brake_pressed, bool standstill, 
              bool cruise_standstill, int v_cruise, float v_target, 
              float v_target_future, float a_target)
  {
    lctl.update(active, v_ego, brake_pressed, standstill, cruise_standstill, 
                v_cruise, v_target, v_target_future, a_target);
  }

}