#ifndef CARPARAMS_H_
#define CARPARAMS_H_
#include <string>

class CarParams
{
public:
    const float LB_TO_KG = 0.453592;
    const float STD_CARGO_KG = 136.0;
    const float MAX_ANGLE = 87.0;

    float MASS = 1326.0 + STD_CARGO_KG;
    float WHEELBASE = 2.70;
    float CENTER_TO_FRONT = WHEELBASE * 0.4;
    float CENTER_TO_REAR = WHEELBASE - CENTER_TO_FRONT;
    int ROTATIONAL_INERTIA = 2500;
    int TIRE_STIFFNESS_FRONT = 192150;
    int TIRE_STIFFNESS_REAR = 202500;

    std::string carName = "ford";
    std::string carFingerprint = "FORD FUSION 2018";
    std::string carVin = "00000000000000000";
    bool isPandaBlack = true;
    int safetyModel = 6;
    bool dashcamOnly = true;

    float radarTimeStep = 0.05; //time delta between radar updates, 20Hz is very standard

    // pedal
    bool enableCruise = true;
    float wheelbase = 2.85;
    float steerRatio = 14.8;
    float mass = 3045.0 * LB_TO_KG + STD_CARGO_KG;
    float lateralTuning_pid_kiBP = 0.0, lateralTuning_pid_kpBP = 0.0;
    float lateralTuning_pid_kpV = 0.01, lateralTuning_pid_kiV = 0.005;     // TODO: tune this
    float lateralTuning_pid_kf = 1.0 / MAX_ANGLE;       // MAX Steer angle to normalize FF
    float steerActuatorDelay = 0.1;      // Default delay, not measured yet
    float steerLimitTimer = 0.8;
    float steerRateCost = 1.0;
    float centerToFront = wheelbase * 0.44;
    float tire_stiffness_factor = 0.5328;

    // min speed to enable ACC. if car can do stop and go, then set enabling speed
    // to a negative value, so it won't matter.
    float minEnableSpeed = -1.0;

    // TODO: get actual value, for now starting with reasonable value for
    // civic and scaling by mass and wheelbase
    float rotationalInertia = ROTATIONAL_INERTIA * mass * wheelbase * wheelbase / (MASS * WHEELBASE * WHEELBASE);

    // TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    
    // mass and CG position, so all cars will have approximately similar dyn behaviors
    float center_to_rear = wheelbase - centerToFront;
    float tireStiffnessFront = (TIRE_STIFFNESS_FRONT * tire_stiffness_factor) * mass / MASS * (center_to_rear / wheelbase) / (CENTER_TO_REAR / WHEELBASE);
    float tireStiffnessRear = (TIRE_STIFFNESS_REAR * tire_stiffness_factor) * mass / MASS * (centerToFront / wheelbase) / (CENTER_TO_FRONT / WHEELBASE);

    // no rear steering, at least on the listed cars above
    float steerRatioRear = 0.0;
    int steerControlType = 1;

    // steer, gas, brake limitations VS speed
    float steerMaxBP[1] = {0.0};      // breakpoints at 1 and 40 kph
    float steerMaxV[1] = {1.0};      // 2/3rd torque allowed above 45 kph
    float gasMaxBP[1] = {0.0};
    float gasMaxV[1] = {0.5};
    float brakeMaxBP[2] = {5.0, 20.0};
    float brakeMaxV[2] = {1.0, 0.8};

    bool enableCamera = false;
    bool openpilotLongitudinalControl = false;

    bool stoppingControl = false;
    float startAccel = 0.0;

    float longitudinalTuning_deadzoneBP[2] = {0.0, 9.0};
    float longitudinalTuning_deadzoneV[2] = {0.0, 0.15};
    float longitudinalTuning_kpBP[3] = {0.0, 5.0, 35.0};
    float longitudinalTuning_kpV[3] = {3.6, 2.4, 1.5};
    float longitudinalTuning_kiBP[2] = {0.0, 35.0};
    float longitudinalTuning_kiV[2] = {0.54, 0.36};
};
#endif         // CARPARAMS_H_