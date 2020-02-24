import os

from cffi import FFI

pptest_dir = os.path.dirname(os.path.abspath(__file__))
libpptest_fn = os.path.join(pptest_dir, "libpptest.so")

ffi = FFI()
ffi.cdef(
"""
void hello();
void calc_states_after_delay(state_t *states, float v_ego, float steer_angle, float curvature_factor, float steer_ratio, float delay);
""")

libpptest = ffi.dlopen(libpptest_fn)


