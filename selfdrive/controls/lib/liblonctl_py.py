import os

from cffi import FFI

lon_dir = os.path.dirname(os.path.abspath(__file__))
liblctl_fn = os.path.join(lon_dir, "liblctl.so")

ffi = FFI()
ffi.cdef("""

float getFinalGas();
float getOutputSteer();
void reset(float v_pid);
float getVpid();

void update(bool active, float v_ego, bool brake_pressed, bool standstill, 
            bool cruise_standstill, int v_cruise, float v_target, 
            float v_target_future, float a_target);
""")

liblctl = ffi.dlopen(liblctl_fn)
