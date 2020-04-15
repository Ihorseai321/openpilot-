#ifndef CARSTATE_H_
#define CARSTATE_H_
#include "kf.h"
#include "parser.h"
#define WHEEL_RADIUS 0.33

class CarState
{
public:
  CarState();
  ~CarState();
  void update(Parser &cp);

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
  int pcm_acc_status;
  bool main_on;
  int lkas_state;
  // TODO: we also need raw driver torque, needed for Assisted Lane Change
  bool steer_override;
  bool steer_error;
  float user_gas;
  bool brake_pressed;
  bool brake_lights;
  bool generic_toggle;
  double v_wheel_fl;
  double v_wheel_fr;
  double v_wheel_rl;
  double v_wheel_rr;
  double v_wheel;
private:
  double dt;
  double x0[2][1];
  double A[2][2];
  double C[2];
  double K[2][1];
};

#endif // CARSTATE_H_