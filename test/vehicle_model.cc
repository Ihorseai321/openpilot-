#include "vehicle_model.h"

#define STD_CARGO_KG 136.0
#define LB_TO_KG 0.453592
  
VehicleModel::VehicleModel()
{
    float MASS = 1326. + STD_CARGO_KG;
    float WHEELBASE = 2.70;
    float CENTER_TO_FRONT = WHEELBASE * 0.4;
    float CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT;
    float ROTATIONAL_INERTIA = 2500;
    float TIRE_STIFFNESS_FRONT = 192150;
    float TIRE_STIFFNESS_REAR = 202500;

    float tire_stiffness_factor = 0.5328;
    float steerRatio = 14.8;

    m = 3045.0 * LB_TO_KG + STD_CARGO_KG; // mass
    l = 2.85; //wheelbase
    j = 2500 * m * l * l / ((1326.0 + STD_CARGO_KG) * 2.70 * 2.70); //rotationalInertia
    aF = l * 0.44; // centerToFront
    aR = l - aF;
    chi = 0.0; // steerRatioRear
    

    cF_orig = (TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * m / MASS * (aR / l) / (CENTER_TO_REAR / WHEELBASE); // tireStiffnessFront
    cR_orig = (TIRE_STIFFNESS_REAR * tire_stiffness_factor) * m / MASS * (aF / l) / (CENTER_TO_FRONT / WHEELBASE); // tireStiffnessRear
    update_params(1.0, steerRatio);
}

VehicleModel::~VehicleModel()
{

}

void VehicleModel::update_params(float stiffness_factor, float steer_ratio)
{
    /*
    Update the vehicle model with a new stiffness factor and steer ratio
    */
    cF = stiffness_factor * cF_orig;
    cR = stiffness_factor * cR_orig;
    sR = steer_ratio;
}

void VehicleModel::steady_state_sol(float sa, float u)
{
    float K[2][1] = {{0.0}, {0.0}};
    if(u > 0.1){
      dyn_ss_sol(sa, u, K);
    }    
    else{
      kin_ss_sol(sa, u, K);
    }
}

float VehicleModel::curvature_factor(float u)
{
    float sf = calc_slip_factor();
    return (1.0 - chi) / (1.0 - sf * u * u) / l;
}

float VehicleModel::calc_curvature(float sa, float u)
{
    return curvature_factor(u) * sa / sR;
}

float VehicleModel::get_steer_from_curvature(float curv, float u)
{
    return curv * sR * 1.0 / curvature_factor(u);
}

float VehicleModel::get_steer_from_yaw_rate(float yaw_rate, float u)
{
    float curv = yaw_rate / u;
    return get_steer_from_curvature(curv, u);
}

float VehicleModel::yaw_rate(float sa, float u)
{
    return calc_curvature(sa, u) * u;
}

void VehicleModel::create_dyn_state_matrices(float u, float A[2][2], float B[2][1])
{
    A[0][0] = - (cF + cR) / (m * u);
    A[0][1] = - (cF * aF - cR * aR) / (m * u) - u;
    A[1][0] = - (cF * aF - cR * aR) / (j * u);
    A[1][1] = - (cF * aF * aF + cR * aR * aR) / (j * u);
    B[0][0] = (cF + chi * cR) / m / sR;
    B[1][0] = (cF * aF - chi * cR * aR) / j / sR;
}

void VehicleModel::kin_ss_sol(float sa, float u, float K[2][1])
{
    K[0][0] = aR / sR / l * u * sa;
    K[1][0] = 1.0 / sR / l * u * sa;
}

void VehicleModel::dyn_ss_sol(float sa, float u, float K[2][1])
{
    float A[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    float B[2][1] = {{0.0}, {0.0}};
    create_dyn_state_matrices(u, A, B);

    K[0][0] = (A[1][1] * B[0][0] - A[0][1] * B[1][0]) / (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
    K[1][0] = (A[1][0] * B[0][0] - A[0][0] * B[1][0]) / (A[1][0] * A[0][1] - A[0][0] * A[1][1]);
    
    K[0][0] = -K[0][0] * sa;
    K[1][0] = -K[1][0] * sa;
}

float VehicleModel::calc_slip_factor()
{
    return m * (cF * aF - cR * aR) / (l * l * cF * cR);
}