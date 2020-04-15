#ifndef LATCONTROL_LQR_H_
#define LATCONTROL_LQR_H_

typedef struct{
  bool active;
  float steerAngle;
  float i;
  float output;
  float lqrOutput;
  bool saturated;
}LateralLQRState;

typedef struct
{
  float output_steer;
  float angle_steers_des;
  LateralLQRState lqr_log;
}LatLQRRet;

class LatControlLQR
{
public:
  LatControlLQR();
  ~LatControlLQR();
  void reset();
  LatLQRRet update(bool active, float v_ego, float angle_steers, 
          float angle_steers_rate, float eps_torque, bool steer_override, 
          bool rate_limited, float path_plan_angleSteers, float path_plan_angleOffset);

  float angle_steers_des;
  float output_steer;
  
  bool lqr_log_active;
  float lqr_log_steerAngle;
  float lqr_log_i;
  float lqr_log_output;
  float lqr_log_lqrOutput;
  bool lqr_log_saturated;

private:
  bool _check_saturation(float control, bool check_saturation, float limit);
  
  float scale;
  float ki;

  float A[4]; //2 * 2
  float B[2]; //2 * 1
  float C[2]; //1 * 2
  float K[2]; //1 * 2
  float L[2]; //2 * 1
  float dc_gain;

  float x_hat[2]; // 2 * 1
  float i_unwind_rate;
  float i_rate;
  float sat_count_rate;
  float sat_limit;
  
  
  float i_lqr;
  
  float sat_count;
};
#endif // LATCONTROL_LQR_H_