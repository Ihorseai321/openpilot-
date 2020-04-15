import os

from cffi import FFI

lqr_dir = os.path.dirname(os.path.abspath(__file__))
liblqr_fn = os.path.join(mpc_dir, "liblqr.so")

ffi = FFI()
ffi.cdef("""
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

float getAngleSteersDes();
  
void reset();
  

LatLQRRet update(bool active, float v_ego, float angle_steers, 
        float angle_steers_rate, float eps_torque, bool steer_override, 
        bool rate_limited, float path_plan_angleSteers, float path_plan_angleOffset);
""")

liblqr = ffi.dlopen(liblqr_fn)
