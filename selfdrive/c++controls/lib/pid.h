#ifndef PID_H_
#define PID_H_
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))

class PIController
{
public:
    PIController(int rate=100, float sat_limit=0.8, bool convert=false, float k_f=1., float pos_limit=0., float neg_limit=0.);
    virtual ~PIController();
    // float get_k_p();
    // float get_k_i();
    void reset();
    bool _check_saturation(float control, bool check_saturation, float error);
    float update(float *kpBP, float *kpV, int size_p, float *kiBP, float *kiV, int size_i, float setpoint, float measurement, float speed, float deadzone, float feedforward, bool freeze_integrator, bool check_saturation, bool override);
    // float kpBP[3]; // proportional gain
    // float kpV[3];
    // float kiBP[2]; // integral gain
    // float kiV[2];
    float k_f;  // feedforward gain

    float pos_limit;
    float neg_limit;

    float sat_count_rate;
    float i_unwind_rate;
    float i_rate;
    float sat_limit;
    float convert;
    float p;
    float i;
    float f;
    bool saturated;
private:
    float apply_deadzone(float error, float deadzone);
    float car_interface_compute_gb(float accel, float speed);

    float speed;
    float sat_count;
    float control;
};

#endif // PID_H_