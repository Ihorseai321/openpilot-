#ifndef LONGCONTROL_H_
#define LONGCONTROL_H_
#include "utils.h"
#include "pid.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "carparams.h"


cereal::ControlsState::LongControlState long_control_state_trans(bool active, cereal::ControlsState::LongControlState long_control_state, float v_ego, float v_target, float v_pid, float output_gb, bool brake_pressed, bool cruise_standstill);

class LongControl
{
public:
    LongControl();
    virtual ~LongControl();
    void reset(float v_pid);
    LCtrlRet update(bool active, float v_ego, bool brake_pressed, bool standstill, bool cruise_standstill, int v_cruise, float v_target, float v_target_future, float a_target);
    
    PIController pid;
    CarParams CP;
    cereal::ControlsState::LongControlState long_control_state;
    float v_pid;
    float last_output_gb;
};
#endif // LONGCONTROL_H_