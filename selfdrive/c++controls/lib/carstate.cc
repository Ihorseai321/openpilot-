#include "carstate.h"

CarState::CarState()
{
  left_blinker_on = 0;
  right_blinker_on = 0; 
  dt = 0.01;
  x0[0][0] = 0.0;
  x0[1][0] = 0.0;
  A[0][0] = 1.0;
  A[0][1] = dt;
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  C[0] = 1.0;
  C[1] = 0.0;
  K[0][0] = 0.12287673;
  K[1][0] = 0.29666309;
  v_ego_kf = new KF1D(x0, A, C, K);

  v_ego = 0.0;
}

CarState::~CarState()
{
  delete v_ego_kf;
}

void CarState::update(Parser cp)
{
  // update prevs, update must run once per loop
  prev_left_blinker_on = left_blinker_on;
  prev_right_blinker_on = right_blinker_on;

  // calc best v_ego estimate, by averaging two opposite corners
  std::pair<std::string, std::string> wheel_speed_pair("WheelSpeed_CG1", "WhlRr_W_Meas");
  v_wheel_fl = cp.vl[wheel_speed_pair] * WHEEL_RADIUS;
  v_wheel_fr = cp.vl[wheel_speed_pair] * WHEEL_RADIUS;
  v_wheel_rl = cp.vl[wheel_speed_pair] * WHEEL_RADIUS;
  v_wheel_rr = cp.vl[wheel_speed_pair] * WHEEL_RADIUS;
  v_wheel = (v_wheel_fl + v_wheel_fr + v_wheel_rl + v_wheel_rr) / 4;

  // Kalman filter
  if(std::abs(v_wheel - v_ego) > 2.0){  // Prevent large accelerations when car starts at non zero speed
    double tmp[2][1];
    tmp[0][0] = v_wheel;
    tmp[1][0] = 0.0;
    v_ego_kf->setX(tmp);
  }

  double v_ego_raw = v_wheel;
  double v_ego_x[2][1] = {{0}, {0}};
  v_ego_kf->update(v_wheel, v_ego_x);
  v_ego = v_ego_x[0][0];
  a_ego = v_ego_x[1][0];

  standstill = !(v_wheel > 0.001);

  std::pair<std::string, std::string> angle_steer_pair("Steering_Wheel_Data_CG1", "SteWhlRelInit_An_Sns");
  angle_steers = (float)cp.vl[angle_steer_pair];

  std::pair<std::string, std::string> v_cruise_pcm_pair("Cruise_Status", "Set_Speed");
  v_cruise_pcm = (float)cp.vl[v_cruise_pcm_pair] * MPH_TO_MS;

  std::pair<std::string, std::string> pcm_acc_status_pair("Cruise_Status", "Cruise_State");
  pcm_acc_status = (int)cp.vl[pcm_acc_status_pair];

  main_on = (bool)cp.vl[pcm_acc_status_pair] != 0;

  std::pair<std::string, std::string> lkas_state_pair("Lane_Keep_Assist_Status", "LaActAvail_D_Actl");
  lkas_state = (int)cp.vl[lkas_state_pair];
  // TODO: we also need raw driver torque, needed for Assisted Lane Change

  std::pair<std::string, std::string> steer_override_pair("Lane_Keep_Assist_Status", "LaHandsOff_B_Actl");
  steer_override = (float)!cp.vl[steer_override_pair];

  std::pair<std::string, std::string> steer_error_pair("Lane_Keep_Assist_Status", "LaActDeny_B_Actl");
  steer_error = (bool)cp.vl[steer_error_pair];

  std::pair<std::string, std::string> user_gas_pair("EngineData_14", "ApedPosScal_Pc_Actl");
  user_gas = (float)cp.vl[user_gas_pair];

  std::pair<std::string, std::string> brake_pressed_pair("Cruise_Status", "Brake_Drv_Appl");
  brake_pressed = (bool)cp.vl[brake_pressed_pair];

  std::pair<std::string, std::string> brake_lights_pair("BCM_to_HS_Body", "Brake_Lights");
  brake_lights = (bool)cp.vl[brake_lights_pair];

  std::pair<std::string, std::string> generic_toggle_pair("Steering_Buttons", "Dist_Incr");
  generic_toggle = (bool)cp.vl[generic_toggle_pair];
}