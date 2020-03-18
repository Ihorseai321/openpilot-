#include "pid.h"
#include "utils.h"


PIController::PIController(float kpBP[], float kpV[], float kiBP[], float kiV[], int rate, float sat_limit, bool convert, float k_f, float pos_limit, float neg_limit)
{
    for(int m = 0; m < ARRAYSIZE(kpBP); ++m){
        this->kpBP[m] = kpBP[m];
        this->kpV[m] = kpV[m];
    }

    for(int j = 0; j < ARRAYSIZE(kiBP); ++j){
        this->kiBP[j] = kiBP[j];
        this->kiV[j] = kiV[j];
    }

    this->k_f = k_f;
    this->pos_limit = pos_limit;
    this->neg_limit = neg_limit;
    sat_count_rate = 1.0 / rate;
    i_unwind_rate = 0.3 / rate;
    i_rate = 1.0 / rate;
    this->sat_limit = sat_limit;
    this->convert = convert;
    reset();
    speed = 0.0;
}

PIController::~PIController()
{}

void PIController::reset()
{
    p = 0.0;
    i = 0.0;
    f = 0.0;
    sat_count_rate = 0.0;
    saturated = false;
    control = 0;
}

float PIController::car_interface_compute_gb(float accel, float speed)
{
    return accel / 3.0;
}

float PIController::get_k_p()
{
    return interp(speed, kpBP, kpV, ARRAYSIZE(kpBP));
}

float PIController::get_k_i()
{
    return interp(speed, kiBP, kiV, ARRAYSIZE(kiBP));
}

float PIController::apply_deadzone(float error, float deadzone)
{
    if(error > deadzone){
        error -= deadzone;
    }
    else if(error < - deadzone){
        error += deadzone;
    }
    else{
        error = 0.0;
    }

    return error;
}

bool PIController::_check_saturation(float control, bool check_saturation, float error)
{
    bool saturated = (control < neg_limit) || (control > pos_limit);
    if(saturated && check_saturation && _fabs(error) > 0.1){
        sat_count += sat_count_rate;
    }
    else{
        sat_count -= sat_count_rate;
    }

    sat_count = clip(sat_count, 0.0, 1.0);

    return sat_count > sat_limit;
}

float PIController::update(float setpoint, float measurement, float speed, float deadzone, float feedforward, bool freeze_integrator, bool check_saturation, bool override)
{
    this->speed = speed;
    float error = apply_deadzone(setpoint - measurement, deadzone);
    p = error * get_k_p();
    f = feedforward * k_f;
    float control = 0.0;
    float _i = 0.0;

    if(override){
        i -= i_unwind_rate * sign(i);
    }
    else{
        _i = i + error * get_k_i() * i_rate;
        control = p + f + _i;

        if(convert){
            control = car_interface_compute_gb(control, speed);
        }

        // Update when changing i will move the control away from the limits
        // or when i will move towards the sign of the error
        if(((error >= 0 && (control <= pos_limit || _i < 0.0)) || 
            (error <= 0 && (control >= neg_limit || _i > 0.0))) && 
            !freeze_integrator){
            i = _i;
        }
    }

    control = p + f + i;
    if(convert){
        control = car_interface_compute_gb(control, speed);
    }

    this->saturated = _check_saturation(control, check_saturation, error);
    this->control = clip(control, neg_limit, pos_limit);

    return this->control;
}