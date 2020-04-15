#include "longcontrol.h"
#include <cmath>

float STOPPING_EGO_SPEED = 0.5;
float MIN_CAN_SPEED = 0.3;  // TODO: parametrize this in car interface
float STOPPING_TARGET_SPEED = MIN_CAN_SPEED + 0.01;
float STARTING_TARGET_SPEED = 0.5;
float BRAKE_THRESHOLD_TO_PID = 0.2;

float STOPPING_BRAKE_RATE = 0.2;  // brake_travel/s while trying to stop
float STARTING_BRAKE_RATE = 0.8;  // brake_travel/s while releasing on restart
float BRAKE_STOPPING_TARGET = 0.5;  // apply at least this amount of brake to maintain the vehicle stationary

float MAX_SPEED_ERROR_BP[2] = {0.0, 30.0}; // speed breakpoints
float MAX_SPEED_ERROR_V[2] = {1.5, 0.8};  // max positive v_pid error VS actual speed; this avoids controls windup due to slow pedal resp

float RATE = 100.0;

float gasMaxBP[1] = {0.};
float gasMaxV[1] = {0.5};
float longitudinalTuning_kpV[3] = {3.6, 2.4, 1.5};
float longitudinalTuning_kiV[2] = {0.54, 0.36};
float longitudinalTuning_kpBP[3] = {0., 5., 35.};
float longitudinalTuning_kiBP[2] = {0., 35.};
float longitudinalTuning_deadzoneBP[2] = {0., 9.};
float longitudinalTuning_deadzoneV[2] = {0., .15};
float brakeMaxBP[1] = {0.};
float brakeMaxV[1] = {1.};
bool stoppingControl = false;

LongControl::LongControl():pid(100.0, 0.8, true)
{
    long_control_state = cereal::ControlsState::LongControlState::OFF;
    v_pid = 0.0;
    last_output_gb = 0.0;
}

LongControl::~LongControl()
{}

void LongControl::reset(float v_pid)
{
    pid.reset();
    this->v_pid = v_pid;
}

void LongControl::update(bool active, float v_ego, bool brake_pressed, bool standstill, bool cruise_standstill, int v_cruise, float v_target, float v_target_future, float a_target)
{
    // """Update longitudinal control. This updates the state machine and runs a PID loop"""
    // Actuation limits
    float gas_max = interp(v_ego, gasMaxBP, gasMaxV, 1);
    float brake_max = interp(v_ego, brakeMaxBP, brakeMaxV, 2);

    // Update state machine
    float output_gb = last_output_gb;
    long_control_state = long_control_state_trans(active, long_control_state, v_ego,
                                                  v_target_future, v_pid, output_gb,
                                                  brake_pressed, cruise_standstill);

    float v_ego_pid = std::max(v_ego, MIN_CAN_SPEED);  // Without this we get jumps, CAN bus reports 0 when speed < 0.3
    if(long_control_state == cereal::ControlsState::LongControlState::OFF){
        v_pid = v_ego_pid;
        pid.reset();
        output_gb = 0.0;
    }
    // tracking objects and driving
    else if(long_control_state == cereal::ControlsState::LongControlState::PID){
        v_pid = v_target;
        pid.pos_limit = gas_max;
        pid.neg_limit = -brake_max;

        // Toyota starts braking more when it thinks you want to stop
        // Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
        bool prevent_overshoot = !stoppingControl && v_ego < 1.5 && v_target_future < 0.7;
        float deadzone = interp(v_ego_pid, longitudinalTuning_deadzoneBP, longitudinalTuning_deadzoneV, 2);

        output_gb = pid.update(longitudinalTuning_kpBP, longitudinalTuning_kpV, 3, longitudinalTuning_kiBP, 
                               longitudinalTuning_kiV, 2, v_pid, v_ego_pid, v_ego_pid, deadzone, a_target, 
                               prevent_overshoot);

        if(prevent_overshoot){
            output_gb = std::min(output_gb, float(0.0));
        }
    }
    // Intention is to stop, switch to a different brake control until we stop
    else if(long_control_state == cereal::ControlsState::LongControlState::STOPPING){
        // Keep applying brakes until the car is stopped
        if(!standstill || output_gb > -BRAKE_STOPPING_TARGET){
            output_gb -= STOPPING_BRAKE_RATE / RATE;
        }
        output_gb = clip(output_gb, -brake_max, gas_max);

        v_pid = v_ego;
        pid.reset();
    }
    // Intention is to move again, release brake fast before handing control to PID
    else if(long_control_state == cereal::ControlsState::LongControlState::STARTING){
        if(output_gb < -0.2){
            output_gb += STARTING_BRAKE_RATE / RATE;
        }
        v_pid = v_ego;
        pid.reset();
    }

    last_output_gb = output_gb;

    final_gas = clip(output_gb, 0.0, gas_max);
    final_brake = -clip(output_gb, -brake_max, 0.0);
}

cereal::ControlsState::LongControlState long_control_state_trans(bool active, cereal::ControlsState::LongControlState long_control_state, float v_ego, float v_target, float v_pid, float output_gb, bool brake_pressed, bool cruise_standstill)
{
    // """Update longitudinal control state machine"""
    bool stopping_condition = (v_ego < 2.0 && cruise_standstill) || (v_ego < STOPPING_EGO_SPEED && ((v_pid < STOPPING_TARGET_SPEED && v_target < STOPPING_TARGET_SPEED) || brake_pressed));

    bool starting_condition = v_target > STARTING_TARGET_SPEED && !cruise_standstill;
    
    if(!active){
        long_control_state = cereal::ControlsState::LongControlState::OFF;
    }
    else{
        if(long_control_state == cereal::ControlsState::LongControlState::OFF){
            if(active){
                long_control_state = cereal::ControlsState::LongControlState::PID;
            }
        }
        else if(long_control_state == cereal::ControlsState::LongControlState::PID){
            if(stopping_condition){
                long_control_state = cereal::ControlsState::LongControlState::STOPPING;
            }
        }
        else if(long_control_state == cereal::ControlsState::LongControlState::STOPPING){
            if(starting_condition){
                long_control_state = cereal::ControlsState::LongControlState::STARTING;
            }
        }
        else if(long_control_state == cereal::ControlsState::LongControlState::STARTING){
            if(stopping_condition){
                long_control_state = cereal::ControlsState::LongControlState::STOPPING;
            }
            else if(output_gb >= -BRAKE_THRESHOLD_TO_PID){
                long_control_state = cereal::ControlsState::LongControlState::PID;
            }
        }
    }

    return long_control_state;
}
