#ifndef VEHICLE_MODEL_H_
#define VEHICLE_MODEL_H_

class VehicleModel
{
public:
    VehicleModel();
    virtual ~VehicleModel();
    void update_params(float stiffness_factor, float steer_ratio);
    void steady_state_sol(float sa, float u);
    float curvature_factor(float u);
    float calc_curvature(float sa, float u);
    float get_steer_from_curvature(float curv, float u);
    float get_steer_from_yaw_rate(float yaw_rate, float u);
    float yaw_rate(float sa, float u);

    float m;
    float j;
    float l;
    float aF;
    float aR;
    float chi;

    float cF_orig;
    float cR_orig;
    float cF;
    float cR;
    float sR;

private:
    void create_dyn_state_matrices(float u, float A[2][2], float B[2][1]);
    void kin_ss_sol(float sa, float u, float K[2][1]);
    void dyn_ss_sol(float sa, float u, float K[2][1]);
    float calc_slip_factor();
};

#endif // VEHICLE_MODEL_H_