import os

from cffi import FFI

lqr_dir = os.path.dirname(os.path.abspath(__file__))
liblqr_fn = os.path.join(lqr_dir, "liblqr.so")

ffi = FFI()
ffi.cdef("""

float getAngleSteersDes();
float getOutputSteer();

bool get_lqr_log_saturated();
float get_lqr_log_lqrOutput();
float get_lqr_log_output();
float get_lqr_log_i();
float get_lqr_log_steerAngle();
bool get_lqr_log_active();
  
void reset();
  

void update(bool active, float v_ego, float angle_steers, 
          float angle_steers_rate, float eps_torque, bool steer_override, 
          bool rate_limited, float path_plan_angleSteers, float path_plan_angleOffset);
""")

liblqr = ffi.dlopen(liblqr_fn)
