#include "carstate.h"

CarState::CarState()
{
  left_blinker_on = 0;
  right_blinker_on = 0; 
  dt = 0.01;
  x0 = {{0.0}, {0.0}};
  A = {{1.0, dt}, {0.0, 1.0}};
  C = {1.0, 0.0};
  K = {{0.12287673}, {0.29666309}};
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
  double v_wheel_fl = cp.vl["WheelSpeed_CG1"]['WhlRr_W_Meas'] * WHEEL_RADIUS;
  double v_wheel_fr = cp.vl["WheelSpeed_CG1"]['WhlRl_W_Meas'] * WHEEL_RADIUS;
  double v_wheel_rl = cp.vl["WheelSpeed_CG1"]['WhlFr_W_Meas'] * WHEEL_RADIUS;
  double v_wheel_rr = cp.vl["WheelSpeed_CG1"]['WhlFl_W_Meas'] * WHEEL_RADIUS;
  double v_wheel = mean([v_wheel_fl, v_wheel_fr, v_wheel_rl, v_wheel_rr]);

  // Kalman filter
  if(abs(v_wheel - v_ego) > 2.0){  // Prevent large accelerations when car starts at non zero speed
    double tmp[2][1];
    tmp[0][0] = v_wheel;
    tmp[1][0] = 0.0;
    v_ego_kf->setX(tmp);
  }

  double v_ego_raw = v_wheel;
  double v_ego_x[2][1] = {{0}, {0}};
  v_ego_kf.update(v_wheel, v_ego_x);
  v_ego = v_ego_x[0][0];
  a_ego = v_ego_x[1][0];
  standstill = !(v_wheel > 0.001);

  angle_steers = cp.vl["Steering_Wheel_Data_CG1"]['SteWhlRelInit_An_Sns'];
  v_cruise_pcm = cp.vl["Cruise_Status"]['Set_Speed'] * MPH_TO_MS;
  pcm_acc_status = cp.vl["Cruise_Status"]['Cruise_State'];
  main_on = cp.vl["Cruise_Status"]['Cruise_State'] != 0;
  lkas_state = cp.vl["Lane_Keep_Assist_Status"]['LaActAvail_D_Actl'];
  // TODO: we also need raw driver torque, needed for Assisted Lane Change
  steer_override = !cp.vl["Lane_Keep_Assist_Status"]['LaHandsOff_B_Actl'];
  steer_error = cp.vl["Lane_Keep_Assist_Status"]['LaActDeny_B_Actl'];
  user_gas = cp.vl["EngineData_14"]['ApedPosScal_Pc_Actl'];
  brake_pressed = cp.vl["Cruise_Status"]["Brake_Drv_Appl"];
  brake_lights = cp.vl["BCM_to_HS_Body"]["Brake_Lights"];
  generic_toggle = cp.vl["Steering_Buttons"]["Dist_Incr"];
}