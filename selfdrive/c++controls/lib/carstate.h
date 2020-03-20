#ifndef CARSTATE_H_
#define CARSTATE_H_
#include "kf.h"
class CarState
{
public:
  CarState();
  ~CarState();
  void update(Parser cp);

  int left_blinker_on;
  int right_blinker_on;
  int prev_left_blinker_on;
  int prev_right_blinker_on; 

  KF1D *v_ego_kf;
  float v_ego;
  float a_ego;
  double v_ego_raw;
  bool standstill;
  float angle_steers;
  float v_cruise_pcm;
  float pcm_acc_status;
  bool main_on;
  int lkas_state;
  // TODO: we also need raw driver torque, needed for Assisted Lane Change
  bool steer_override;
  bool steer_error;
  float user_gas;
  bool brake_pressed;
  bool brake_lights;
  bool generic_toggle;
private:
  double dt;
  double x0;
  double A;
  double C;
  double K;
};

#endif // CARSTATE_H_